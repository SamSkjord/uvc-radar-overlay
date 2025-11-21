#!/usr/bin/env python3
"""
Reusable Toyota radar driver with CAN interface configuration, keep-alive
management, and callback hooks for downstream consumers.
"""

from __future__ import annotations

import dataclasses
import os
import subprocess
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional

import can
import cantools


TRACK_BASE_ID = 0x210
TRACK_MAX_ID = 0x21F


class ECU:
    CAM = 0
    DSU = 1
    APGS = 2


STATIC_MSGS = [
    (0x141, ECU.DSU, 1, 2, b"\x00\x00\x00\x46"),
    (0x128, ECU.DSU, 1, 3, b"\xf4\x01\x90\x83\x00\x37"),
    (0x283, ECU.DSU, 0, 3, b"\x00\x00\x00\x00\x00\x00\x8c"),
    (0x344, ECU.DSU, 0, 5, b"\x00\x00\x01\x00\x00\x00\x00\x50"),
    (0x160, ECU.DSU, 1, 7, b"\x00\x00\x08\x12\x01\x31\x9c\x51"),
    (0x161, ECU.DSU, 1, 7, b"\x00\x1e\x00\x00\x00\x80\x07"),
    (0x365, ECU.DSU, 0, 20, b"\x00\x00\x00\x80\xfc\x00\x08"),
    (0x366, ECU.DSU, 0, 20, b"\x00\x72\x07\xff\x09\xfe\x00"),
    (0x4CB, ECU.DSU, 0, 100, b"\x0c\x00\x00\x00\x00\x00\x00\x00"),
]


@dataclasses.dataclass
class ToyotaRadarConfig:
    radar_channel: str = "can1"
    car_channel: str = "can0"
    interface: str = "socketcan"
    radar_interface: Optional[str] = None
    car_interface: Optional[str] = None
    bitrate: int = 500000
    radar_dbc: str = "opendbc/toyota_prius_2017_adas.dbc"
    control_dbc: str = "opendbc/toyota_prius_2017_pt_generated.dbc"
    keepalive_rate_hz: float = 100.0
    track_timeout: float = 0.5
    notifier_timeout: float = 0.1
    auto_setup: bool = True
    use_sudo: bool = False
    setup_extra_args: Iterable[str] = dataclasses.field(default_factory=list)
    keepalive_enabled: bool = True


@dataclasses.dataclass
class RadarTrack:
    track_id: int
    long_dist: float
    lat_dist: float
    rel_speed: float
    new_track: int
    timestamp: float
    raw: Dict[str, float]


TrackCallback = Callable[[RadarTrack], None]
RawMessageCallback = Callable[[can.Message], None]


class RadarKeepAlive(threading.Thread):
    """Send static DSU/ACC frames to keep the radar streaming tracks."""

    def __init__(
        self,
        car_bus: can.BusABC,
        radar_bus: can.BusABC,
        control_db,
        rate_hz: float,
    ) -> None:
        super().__init__(daemon=True, name="ToyotaRadarKeepAlive")
        self._car_bus = car_bus
        self._radar_bus = radar_bus
        self._control_db = control_db
        self._period = 1.0 / max(rate_hz, 1.0)
        self._stop_event = threading.Event()
        self.tx_count = 0
        self.last_error: Optional[str] = None
        self._acc_message = control_db.get_message_by_name("ACC_CONTROL")
        self._frame = 0

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:  # pragma: no cover - requires hardware
        while not self._stop_event.is_set():
            start = time.time()
            try:
                self._send_frame()
                self.last_error = None
            except Exception as exc:
                self.last_error = str(exc)
            self._frame += 1
            elapsed = time.time() - start
            remaining = self._period - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)

    def _send_frame(self) -> None:
        if self._acc_message:
            payload = self._acc_message.encode(
                {
                    "ACCEL_CMD": 0.0,
                    "SET_ME_X63": 0x63,
                    "SET_ME_1": 1,
                    "RELEASE_STANDSTILL": 1,
                    "CANCEL_REQ": 0,
                    "CHECKSUM": 113,
                }
            )
            self._car_bus.send(
                can.Message(
                    arbitration_id=self._acc_message.frame_id,
                    data=payload,
                    is_extended_id=False,
                )
            )
            self.tx_count += 1

        for addr, _ecu, bus_sel, step, payload in STATIC_MSGS:
            if self._frame % step != 0:
                continue

            data = bytearray(payload)
            if addr in (0x489, 0x48A) and bus_sel == 0:
                cnt = int((self._frame / 100) % 0xF) + 1
                if addr == 0x48A:
                    cnt |= 1 << 7
                data.append(cnt)

            bus = self._car_bus if bus_sel == 0 else self._radar_bus
            bus.send(
                can.Message(arbitration_id=addr, data=bytes(data), is_extended_id=False)
            )
            self.tx_count += 1


class _TrackListener(can.Listener):
    def __init__(self, driver: "ToyotaRadarDriver") -> None:
        self._driver = driver

    def on_message_received(self, msg: can.Message) -> None:  # pragma: no cover
        self._driver._handle_message(msg)


class ToyotaRadarDriver:
    """High-level interface for initializing and consuming Toyota radar tracks."""

    def __init__(self, config: Optional[ToyotaRadarConfig] = None) -> None:
        self.config = config or ToyotaRadarConfig()
        self._car_bus: Optional[can.BusABC] = None
        self._radar_bus: Optional[can.BusABC] = None
        self._notifier: Optional[can.Notifier] = None
        self._buffered_reader: Optional[can.BufferedReader] = None
        self._track_db = None
        self._control_db = None
        self._keepalive: Optional[RadarKeepAlive] = None
        self._track_callbacks: List[TrackCallback] = []
        self._raw_callbacks: List[RawMessageCallback] = []
        self._tracks: Dict[int, RadarTrack] = {}
        self._lock = threading.Lock()
        self._running = False
        self._rx_count = 0

    # --- public API -----------------------------------------------------
    def register_track_callback(self, callback: TrackCallback) -> None:
        self._track_callbacks.append(callback)

    def register_raw_callback(self, callback: RawMessageCallback) -> None:
        self._raw_callbacks.append(callback)

    def start(self) -> None:  # pragma: no cover - hardware dependent
        if self._running:
            return

        if self.config.auto_setup:
            self._setup_interfaces()

        self._track_db = self._load_dbc(self.config.radar_dbc, "Radar")
        self._control_db = self._load_dbc(self.config.control_dbc, "Control")

        car_if = self.config.car_interface or self.config.interface
        radar_if = self.config.radar_interface or self.config.interface

        self._car_bus = can.interface.Bus(
            channel=self.config.car_channel,
            bustype=car_if,
            bitrate=self.config.bitrate,
        )
        self._radar_bus = can.interface.Bus(
            channel=self.config.radar_channel,
            bustype=radar_if,
            bitrate=self.config.bitrate,
        )

        self._send_initial_messages()

        if self.config.keepalive_enabled:
            self._keepalive = RadarKeepAlive(
                self._car_bus,
                self._radar_bus,
                self._control_db,
                rate_hz=self.config.keepalive_rate_hz,
            )
            self._keepalive.start()

        listener = _TrackListener(self)
        # MEMORY FIX: Removed BufferedReader that was accumulating messages infinitely
        # The BufferedReader was never used and caused ~30k Message objects/minute leak
        self._notifier = can.Notifier(
            self._radar_bus,
            [listener] + self._raw_callbacks,
            timeout=self.config.notifier_timeout,
        )
        self._buffered_reader = None  # Keep for compatibility but don't create
        self._rx_count = 0
        self._running = True

    def stop(self) -> None:  # pragma: no cover - hardware dependent
        if not self._running:
            return

        if self._notifier:
            self._notifier.stop()

        if self._keepalive:
            self._keepalive.stop()
            self._keepalive.join(timeout=1.0)

        if self._radar_bus:
            self._radar_bus.shutdown()
        if self._car_bus:
            self._car_bus.shutdown()

        self._running = False
        self._buffered_reader = None

    def get_tracks(self) -> Dict[int, RadarTrack]:
        cutoff = time.time() - self.config.track_timeout
        with self._lock:
            stale = [track_id for track_id, track in self._tracks.items() if track.timestamp < cutoff]
            for track_id in stale:
                self._tracks.pop(track_id, None)
            return dict(self._tracks)

    def keepalive_status(self) -> Optional[Dict[str, Optional[float]]]:
        if not self._keepalive:
            return None
        return {"tx_count": self._keepalive.tx_count, "last_error": self._keepalive.last_error}

    def message_count(self) -> int:
        return self._rx_count

    # --- internals ------------------------------------------------------
    def _handle_message(self, msg: can.Message) -> None:
        self._rx_count += 1
        if not (
            TRACK_BASE_ID <= msg.arbitration_id <= TRACK_MAX_ID
            and self._track_db is not None
        ):
            for cb in self._raw_callbacks:
                cb(msg)
            return

        try:
            decoded = self._track_db.decode_message(msg.arbitration_id, msg.data)
        except Exception:
            return

        if decoded.get("VALID", 0) != 1:
            return

        track_id = msg.arbitration_id - TRACK_BASE_ID
        track = RadarTrack(
            track_id=track_id,
            long_dist=decoded.get("LONG_DIST", 0.0),
            lat_dist=decoded.get("LAT_DIST", 0.0),
            rel_speed=decoded.get("REL_SPEED", 0.0),
            new_track=decoded.get("NEW_TRACK", 0),
            timestamp=time.time(),
            raw=decoded,
        )

        with self._lock:
            self._tracks[track_id] = track

        for callback in self._track_callbacks:
            try:
                callback(track)
            except Exception:
                pass

    def _load_dbc(self, path: str, label: str):
        if not os.path.exists(path):
            raise FileNotFoundError(f"{label} DBC not found: {path}")
        return cantools.database.load_file(path, strict=False)

    def _send_initial_messages(self) -> None:
        if not self._control_db or not self._car_bus:
            return

        init_msgs = (
            ("SPEED", {"ENCODER": 0, "SPEED": 1.44, "CHECKSUM": 0}),
            (
                "PCM_CRUISE",
                {
                    "CRUISE_STATE": 9,
                    "GAS_RELEASED": 0,
                    "STANDSTILL_ON": 0,
                    "ACCEL_NET": 0,
                    "CHECKSUM": 0,
                },
            ),
            (
                "PCM_CRUISE_2",
                {
                    "MAIN_ON": 0,
                    "LOW_SPEED_LOCKOUT": 0,
                    "SET_SPEED": 0,
                    "CHECKSUM": 0,
                },
            ),
            (
                "ACC_CONTROL",
                {
                    "ACCEL_CMD": 0,
                    "SET_ME_X63": 0,
                    "RELEASE_STANDSTILL": 0,
                    "SET_ME_1": 0,
                    "CANCEL_REQ": 0,
                    "CHECKSUM": 0,
                },
            ),
            (
                "PCM_CRUISE_SM",
                {
                    "MAIN_ON": 0,
                    "CRUISE_CONTROL_STATE": 0,
                    "UI_SET_SPEED": 0,
                },
            ),
        )

        for name, values in init_msgs:
            message = self._control_db.get_message_by_name(name)
            if not message:
                continue
            try:
                payload = message.encode(values)
                self._car_bus.send(
                    can.Message(
                        arbitration_id=message.frame_id,
                        data=payload,
                        is_extended_id=False,
                    )
                )
            except Exception:
                continue

    def _setup_interfaces(self) -> None:
        for channel in {self.config.car_channel, self.config.radar_channel}:
            self._bring_up_interface(channel)

    def _bring_up_interface(self, channel: str) -> None:
        args = [*self.config.setup_extra_args, "ip", "link", "set", channel, "type", "can", "bitrate", str(self.config.bitrate)]
        self._run_command(args, ignore_errors=True)
        args = [*self.config.setup_extra_args, "ip", "link", "set", channel, "up"]
        self._run_command(args, ignore_errors=False)

    def _run_command(self, args: List[str], *, ignore_errors: bool) -> None:
        cmd = []
        if self.config.use_sudo:
            cmd.append("sudo")
        cmd.extend(args)
        try:
            subprocess.run(
                cmd,
                check=not ignore_errors,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError:
            if not ignore_errors:
                raise
