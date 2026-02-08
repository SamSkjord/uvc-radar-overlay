#!/usr/bin/env python3
"""
Reusable Tesla radar driver with CAN interface configuration, keep-alive
management, and callback hooks for downstream consumers.

Drop-in compatible with toyota-radar's ToyotaRadarDriver API:
  - register_track_callback / register_raw_callback
  - start / stop
  - get_tracks / keepalive_status / message_count

Uses the Tesla Bosch MRRevo14F radar protocol (30+ CAN message types at
100 Hz) and decodes 32 object tracks from 0x310-0x36E.
"""

from __future__ import annotations

import dataclasses
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional

import can

from tesla_radar_protocol import (
    TeslaRadarProtocol,
    _load_radar_dbc,
    setup_can,
)
from uds_can import IsoTpSession, IsoTpError


# --- Object frame layout ------------------------------------------------
NUM_OBJECTS = 32
OBJ_BASE = 0x310
OBJ_A_IDS = frozenset(OBJ_BASE + i * 3 for i in range(NUM_OBJECTS))
OBJ_B_IDS = frozenset(OBJ_BASE + i * 3 + 1 for i in range(NUM_OBJECTS))

# UDS addressing for VIN auto-read
UDS_TX = 0x641
UDS_RX = 0x651


# --- Data model ----------------------------------------------------------

@dataclasses.dataclass
class RadarTrack:
    """Base radar track — compatible with toyota-radar's RadarTrack."""
    track_id: int
    long_dist: float
    lat_dist: float
    rel_speed: float
    new_track: int
    timestamp: float
    raw: Dict[str, float]


@dataclasses.dataclass
class TeslaRadarTrack(RadarTrack):
    """Extended track with Tesla-specific typed fields.

    Code that accepts RadarTrack still works (LSP-compatible subclass).
    All extras are also present in the ``raw`` dict.
    """
    prob_exist: float = 0.0
    long_accel: float = 0.0
    lat_speed: float = 0.0
    object_class: int = 0
    moving_state: int = 0
    tracked: int = 0
    valid: int = 0
    meas: int = 0
    prob_obstacle: float = 0.0
    length: float = 0.0
    dz: float = 0.0


TrackCallback = Callable[[RadarTrack], None]
RawMessageCallback = Callable[[can.Message], None]


# --- Config --------------------------------------------------------------

@dataclasses.dataclass
class TeslaRadarConfig:
    channel: str = "can0"
    interface: str = "socketcan"
    bitrate: int = 500000
    radar_dbc: str = "opendbc/tesla_radar.dbc"
    vin: Optional[str] = None
    track_timeout: float = 0.5
    auto_vin: bool = True
    auto_setup: bool = True
    use_sudo: bool = False
    setup_extra_args: Iterable[str] = dataclasses.field(default_factory=list)
    debug: bool = False


# --- Driver --------------------------------------------------------------

class TeslaRadarDriver:
    """High-level interface for initializing and consuming Tesla radar tracks.

    Public API matches ToyotaRadarDriver exactly so the same consumer code
    (callbacks, curses visualization) works with either radar.
    """

    def __init__(self, config: Optional[TeslaRadarConfig] = None) -> None:
        self.config = config or TeslaRadarConfig()
        self._bus: Optional[can.BusABC] = None
        self._protocol: Optional[TeslaRadarProtocol] = None
        self._proto_thread: Optional[threading.Thread] = None
        self._track_db = None
        self._track_callbacks: List[TrackCallback] = []
        self._raw_callbacks: List[RawMessageCallback] = []
        self._tracks: Dict[int, TeslaRadarTrack] = {}
        self._seen_ids: set = set()
        self._lock = threading.Lock()
        self._running = False
        self._rx_count = 0

        # Pre-computed DBC message definitions for object frames
        self._a_msg_def: Dict[int, object] = {}
        self._b_msg_def: Dict[int, object] = {}
        # Stashed A-frames waiting for their B-frame pair
        self._a_frames: Dict[int, dict] = {}

    # --- public API (matches ToyotaRadarDriver) -------------------------

    def register_track_callback(self, callback: TrackCallback) -> None:
        self._track_callbacks.append(callback)

    def register_raw_callback(self, callback: RawMessageCallback) -> None:
        self._raw_callbacks.append(callback)

    def start(self) -> None:
        if self._running:
            return

        # 1. Open CAN bus
        self._bus = setup_can(
            channel=self.config.channel,
            bitrate=self.config.bitrate,
            can_interface=self.config.interface,
            auto_setup=self.config.auto_setup,
            use_sudo=self.config.use_sudo,
            setup_extra_args=self.config.setup_extra_args,
        )

        # 2. Auto-read VIN via UDS (bus is quiet, no contention)
        vin = self.config.vin
        if vin is None and self.config.auto_vin:
            try:
                vin = self._read_vin()
            except Exception as exc:
                if self.config.debug:
                    print(f"VIN auto-read failed ({exc}), using default")
                vin = None

        # 3. Load DBC for object decoding
        self._track_db = _load_radar_dbc(self.config.radar_dbc)
        self._build_msg_defs()

        # 4. Create protocol
        proto_kwargs = {"debug": self.config.debug}
        if vin is not None:
            proto_kwargs["vin"] = vin
        self._protocol = TeslaRadarProtocol(self._bus, **proto_kwargs)

        # 5. Patch the RX monitor to decode objects + call our callbacks
        self._protocol.monitor_radar_responses = self._rx_handler

        # 6. Start protocol in background thread
        self._proto_thread = threading.Thread(
            target=self._protocol.start, daemon=True
        )
        self._proto_thread.start()

        self._running = True

    def stop(self) -> None:
        if not self._running:
            return

        if self._protocol:
            self._protocol.stop()

        # Give TX loop time to exit
        if self._proto_thread:
            self._proto_thread.join(timeout=2.0)

        if self._bus:
            self._bus.shutdown()

        self._running = False

    def get_tracks(self) -> Dict[int, RadarTrack]:
        cutoff = time.time() - self.config.track_timeout
        with self._lock:
            stale = [tid for tid, t in self._tracks.items() if t.timestamp < cutoff]
            for tid in stale:
                self._tracks.pop(tid, None)
                self._seen_ids.discard(tid)
            return dict(self._tracks)

    def keepalive_status(self) -> Optional[Dict[str, Optional[float]]]:
        if not self._protocol:
            return None
        return {
            "tx_count": self._protocol.tx_count,
            "last_error": None,
            "radar_status": self._protocol.tesla_radar_status,
            "vin_complete": self._protocol.tesla_radar_vin_complete,
        }

    def message_count(self) -> int:
        return self._rx_count

    # --- internals -------------------------------------------------------

    def _read_vin(self) -> str:
        tp = IsoTpSession(self._bus, UDS_TX, UDS_RX, timeout=1.0)
        tp.send(bytes([0x22, 0xF1, 0x90]))
        resp = tp.recv()
        if resp and resp[0] == 0x62 and len(resp) >= 19:
            return resp[3:20].decode("ascii", errors="replace").strip("\x00 ")
        if resp and resp[0] == 0x7F:
            raise IsoTpError(f"UDS negative response reading VIN: {resp.hex()}")
        raise IsoTpError(f"Unexpected UDS response: {resp.hex() if resp else 'None'}")

    def _build_msg_defs(self):
        """Pre-load DBC message definitions for all 32 object A/B frames."""
        if self._track_db is None:
            return
        for i in range(NUM_OBJECTS):
            a_id = OBJ_BASE + i * 3
            b_id = OBJ_BASE + i * 3 + 1
            try:
                self._a_msg_def[a_id] = self._track_db.get_message_by_frame_id(a_id)
            except KeyError:
                self._a_msg_def[a_id] = self._track_db.get_message_by_frame_id(OBJ_BASE)
            try:
                self._b_msg_def[b_id] = self._track_db.get_message_by_frame_id(b_id)
            except KeyError:
                self._b_msg_def[b_id] = self._track_db.get_message_by_frame_id(OBJ_BASE + 1)

    def _rx_handler(self):
        """Combined RX handler: protocol state tracking + object decoding."""
        protocol = self._protocol

        while protocol.running:
            try:
                msg = protocol.can_bus.recv(timeout=0.1)
            except (ValueError, IndexError):
                continue
            except can.CanError:
                time.sleep(0.1)
                continue

            if msg is None:
                if (
                    time.time() - protocol.last_radar_signal > 1.0
                    and protocol.tesla_radar_status > 0
                ):
                    protocol.tesla_radar_status = 0
                continue

            self._rx_count += 1
            mid = msg.arbitration_id

            # -- Protocol state tracking (0x631, 0x300, 0x3FF) --
            if mid == 0x631:
                if protocol.tesla_radar_status == 0:
                    protocol.tesla_radar_status = 1
                    protocol.last_radar_signal = time.time()
                elif protocol.tesla_radar_status > 0:
                    protocol.last_radar_signal = time.time()
                protocol.init_message_count += 1
                protocol.last_init_data = bytes(msg.data)

            elif mid == 0x300:
                if protocol.tesla_radar_status < 2:
                    protocol.tesla_radar_status = 2
                protocol.last_radar_signal = time.time()
                protocol.status_message_count += 1
                protocol.last_status_data = bytes(msg.data)

            elif mid == 0x3FF:
                protocol.last_system_status_data = bytes(msg.data)
                if len(msg.data) >= 2 and msg.data[1] != 0:
                    protocol.error_code_counts[msg.data[1]] += 1

            # -- Object A-frame decoding --
            elif mid in OBJ_A_IDS:
                obj_idx = (mid - OBJ_BASE) // 3
                try:
                    self._a_frames[obj_idx] = self._a_msg_def[mid].decode(msg.data)
                except Exception:
                    pass

            # -- Object B-frame decoding (pairs with stashed A-frame) --
            elif mid in OBJ_B_IDS:
                obj_idx = (mid - OBJ_BASE) // 3
                try:
                    b_data = self._b_msg_def[mid].decode(msg.data)
                except Exception:
                    continue
                a_data = self._a_frames.get(obj_idx)
                if a_data is None:
                    continue
                self._process_object(obj_idx, a_data, b_data)

            # -- Raw callbacks for anything else --
            else:
                for cb in self._raw_callbacks:
                    try:
                        cb(msg)
                    except Exception:
                        pass

    def _process_object(self, obj_idx: int, a_data: dict, b_data: dict):
        """Build a TeslaRadarTrack from decoded A+B frames and dispatch."""
        prob = a_data.get("ProbExist", 0.0)
        valid = int(a_data.get("Valid", 0))

        # Validity filter: skip if no probability and not valid
        if prob == 0 and valid == 0:
            return

        # Synthesize new_track flag
        is_new = 0
        if obj_idx not in self._seen_ids:
            is_new = 1
            self._seen_ids.add(obj_idx)

        # Build raw dict with all decoded signals
        raw = {}
        raw.update(a_data)
        raw.update(b_data)

        track = TeslaRadarTrack(
            track_id=obj_idx,
            long_dist=a_data.get("LongDist", 0.0),
            lat_dist=a_data.get("LatDist", 0.0),
            rel_speed=a_data.get("LongSpeed", 0.0),
            new_track=is_new,
            timestamp=time.time(),
            raw=raw,
            prob_exist=prob,
            long_accel=a_data.get("LongAccel", 0.0),
            lat_speed=b_data.get("LatSpeed", 0.0),
            object_class=int(b_data.get("Class", 0)),
            moving_state=int(b_data.get("MovingState", 0)),
            tracked=int(a_data.get("Tracked", 0)),
            valid=valid,
            meas=int(a_data.get("Meas", 0)),
            prob_obstacle=b_data.get("ProbObstacle", 0.0),
            length=b_data.get("Length", 0.0),
            dz=b_data.get("dz", 0.0),
        )

        with self._lock:
            self._tracks[obj_idx] = track

        for cb in self._track_callbacks:
            try:
                cb(track)
            except Exception:
                pass
