#!/usr/bin/env python3
"""
Pygame overlay that composites Toyota radar tracks on top of a UVC camera feed.
The overlay highlights the nearest tracks within the configured camera field of
view by drawing green arrows along the top edge of the display.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pygame
import pygame.camera

try:
    import cv2  # type: ignore[import]
except ImportError:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore[assignment]

from toyota_radar_driver import RadarTrack, ToyotaRadarConfig, ToyotaRadarDriver

# Default overlay styling
ARROW_HEIGHT = 40
ARROW_HALF_WIDTH = 18
ARROW_MARGIN_TOP = 12
CHEVRON_INNER_OFFSET = 12
CHEVRON_TIP_INSET = 8
MARKER_COLOR_GREEN = (0, 200, 0)
MARKER_COLOR_YELLOW = (255, 220, 0)
MARKER_COLOR_RED = (255, 0, 0)
TEXT_OFFSET_TOP = ARROW_MARGIN_TOP + ARROW_HEIGHT + 6
TEXT_SPACING = 2
TEXT_COLOR = (255, 255, 255)
SPEED_COLOR_AWAY = (0, 255, 0)
SPEED_COLOR_CLOSING = (255, 0, 0)
SPEED_COLOR_STATIONARY = (200, 200, 200)

OVERTAKE_ARROW_COLOR = (30, 144, 255)
OVERTAKE_ARROW_ALPHA = 230
OVERTAKE_ARROW_WIDTH = 160
OVERTAKE_ARROW_HEIGHT = 100
OVERTAKE_ARROW_MARGIN = 24


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Overlay Toyota radar tracks onto a UVC camera feed using pygame."
    )
    # CAN / radar configuration
    parser.add_argument("--radar-channel", default="can0", help="Radar CAN channel.")
    parser.add_argument("--car-channel", default="can1", help="Car CAN channel.")
    parser.add_argument(
        "--interface",
        default="socketcan",
        help="Default python-can interface for both channels.",
    )
    parser.add_argument(
        "--radar-interface",
        default=None,
        help="Override python-can interface for radar channel.",
    )
    parser.add_argument(
        "--car-interface",
        default=None,
        help="Override python-can interface for car channel.",
    )
    parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate.")
    parser.add_argument(
        "--radar-dbc",
        default="opendbc/toyota_prius_2017_adas.dbc",
        help="DBC used to decode radar tracks.",
    )
    parser.add_argument(
        "--control-dbc",
        default="opendbc/toyota_prius_2017_pt_generated.dbc",
        help="DBC containing ACC/DSU keep-alive messages.",
    )
    parser.add_argument(
        "--no-setup",
        action="store_true",
        help="Skip bringing interfaces up with ip link.",
    )
    parser.add_argument(
        "--use-sudo",
        action="store_true",
        help="Run interface setup commands with sudo.",
    )
    parser.add_argument(
        "--setup-extra",
        action="append",
        default=[],
        metavar="TOKEN",
        help="Extra tokens to prefix ip link commands (repeatable).",
    )
    parser.add_argument(
        "--no-keepalive",
        action="store_true",
        help="Disable internal keep-alive loop.",
    )
    parser.add_argument(
        "--keepalive-rate-hz",
        type=float,
        default=100.0,
        help="Frequency for radar keep-alive loop.",
    )
    parser.add_argument(
        "--track-timeout",
        type=float,
        default=0.5,
        help="Seconds before removing stale tracks from the driver cache.",
    )
    parser.add_argument(
        "--notifier-timeout",
        type=float,
        default=0.1,
        help="python-can notifier timeout in seconds.",
    )
    # Display / camera configuration
    parser.add_argument(
        "--camera-device",
        default="/dev/video0",
        help="Path to the UVC camera device.",
    )
    parser.add_argument("--camera-width", type=int, default=1280, help="Capture width.")
    parser.add_argument(
        "--camera-height", type=int, default=720, help="Capture height."
    )
    parser.add_argument(
        "--camera-fps", type=int, default=30, help="Capture frame rate."
    )
    parser.add_argument(
        "--camera-fov",
        type=float,
        default=106.0,  # 60.0,
        help="Horizontal field of view in degrees for mapping radar tracks.",
    )
    parser.add_argument(
        "--display-width",
        type=int,
        default=None,
        help="Override display width (defaults to camera width).",
    )
    parser.add_argument(
        "--display-height",
        type=int,
        default=None,
        help="Override display height (defaults to camera height).",
    )
    parser.add_argument(
        "--windowed",
        action="store_true",
        help="Use a window instead of fullscreen display mode.",
    )
    parser.add_argument(
        "--mirror-output",
        dest="mirror_output",
        action="store_true",
        default=True,
        help="Horizontally mirror the camera feed and overlay positions (default).",
    )
    parser.add_argument(
        "--no-mirror-output",
        dest="mirror_output",
        action="store_false",
        help="Disable horizontal mirroring of the camera feed and overlay positions.",
    )
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=30.0,
        help="Target UI refresh rate.",
    )
    parser.add_argument(
        "--track-count",
        type=int,
        default=3,
        help="Number of nearest tracks to project onto the display.",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=120.0,
        help="Maximum longitudinal distance in meters to consider for overlay.",
    )
    parser.add_argument(
        "--merge-radius",
        type=float,
        default=1.0,
        help="Merge tracks within this 2D radius (meters) into a single target.",
    )
    parser.add_argument(
        "--warn-yellow-kph",
        type=float,
        default=10.0,
        help="Delta speed threshold (km/h) for switching marker color to yellow.",
    )
    parser.add_argument(
        "--warn-red-kph",
        type=float,
        default=20.0,
        help="Delta speed threshold (km/h) for switching marker color to red.",
    )
    parser.add_argument(
        "--overtake-time-threshold",
        type=float,
        default=1.0,
        help="Maximum time-to-overtake (seconds) to trigger the overtake warning.",
    )
    parser.add_argument(
        "--overtake-min-closing-kph",
        type=float,
        default=5.0,
        help="Minimum closing speed in km/h required to trigger the overtake warning.",
    )
    parser.add_argument(
        "--overtake-min-lateral",
        type=float,
        default=0.5,
        help="Minimum absolute lateral offset in meters to qualify for an overtake warning.",
    )
    parser.add_argument(
        "--overtake-arrow-duration",
        type=float,
        default=1.0,
        help="Seconds to continue displaying the overtake arrow after a track disappears.",
    )
    parser.add_argument(
        "--replay-session",
        type=Path,
        default=None,
        help="Directory containing recorded data from radar_capture.py.",
    )
    parser.add_argument(
        "--replay-video",
        type=Path,
        default=None,
        help="Explicit path to a recorded video file for replay.",
    )
    parser.add_argument(
        "--replay-tracks",
        type=Path,
        default=None,
        help="Explicit path to a recorded track JSONL file for replay.",
    )
    parser.add_argument(
        "--replay-loop",
        action="store_true",
        help="Loop playback when replay inputs reach the end.",
    )
    return parser.parse_args()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class RadarCameraOverlay:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.display_size = (
            args.display_width or args.camera_width,
            args.display_height or args.camera_height,
        )
        self.screen: pygame.Surface | None = None
        self.camera: pygame.camera.Camera | None = None
        self.driver: ToyotaRadarDriver | None = None
        self.clock = pygame.time.Clock()
        self._frame_surface: pygame.Surface | None = None
        self.font: pygame.font.Font | None = None
        self._arrow_cache: Dict[Tuple[int, int, int], pygame.Surface] = {}
        self._overtake_surfaces: Dict[str, pygame.Surface] = {}
        self._overtake_alert: Optional[dict] = None
        self.replay_source: Optional["ReplaySource"] = None
        self._camera_initialized = False
        self._last_debug_print = 0.0

    def _init_pygame(self) -> None:
        pygame.init()
        pygame.display.set_caption("Toyota Radar Camera Overlay")

        if self.args.windowed:
            self.screen = pygame.display.set_mode(self.display_size)
        else:
            try:
                self.screen = pygame.display.set_mode(
                    self.display_size, pygame.FULLSCREEN
                )
            except pygame.error:
                print("Fullscreen mode failed, falling back to windowed mode")
                self.screen = pygame.display.set_mode(self.display_size)

        if self.screen is None:
            raise RuntimeError("Failed to create pygame display surface.")

        self._frame_surface = pygame.Surface(self.display_size)
        pygame.font.init()
        self.font = pygame.font.Font(None, 28)

    def _init_camera(self) -> None:
        pygame.camera.init()
        camera = pygame.camera.Camera(
            self.args.camera_device,
            (self.args.camera_width, self.args.camera_height),
            "RGB",
        )
        try:
            camera.start()
        except Exception as exc:
            raise RuntimeError(
                f"Failed to start camera {self.args.camera_device}: {exc}"
            )
        self.camera = camera
        self._camera_initialized = True

    def _init_driver(self) -> None:
        config = ToyotaRadarConfig(
            radar_channel=self.args.radar_channel,
            car_channel=self.args.car_channel,
            interface=self.args.interface,
            radar_interface=self.args.radar_interface,
            car_interface=self.args.car_interface,
            bitrate=self.args.bitrate,
            radar_dbc=self.args.radar_dbc,
            control_dbc=self.args.control_dbc,
            keepalive_rate_hz=self.args.keepalive_rate_hz,
            track_timeout=self.args.track_timeout,
            notifier_timeout=self.args.notifier_timeout,
            auto_setup=not self.args.no_setup,
            use_sudo=self.args.use_sudo,
            setup_extra_args=self.args.setup_extra,
            keepalive_enabled=not self.args.no_keepalive,
        )
        driver = ToyotaRadarDriver(config)
        driver.start()
        self.driver = driver

    def _replay_enabled(self) -> bool:
        return bool(
            self.args.replay_session
            or self.args.replay_video
            or self.args.replay_tracks
        )

    def _init_replay_source(self) -> None:
        if cv2 is None:
            raise RuntimeError(
                "Replay mode requires opencv-python. Install with "
                "`python3 -m pip install opencv-python`."
            )
        video_path, tracks_path = self._resolve_replay_paths()
        self.replay_source = ReplaySource(video_path, tracks_path)

        width = self.args.display_width or self.replay_source.width
        height = self.args.display_height or self.replay_source.height
        self.display_size = (width, height)

    def _resolve_replay_paths(self) -> Tuple[Path, Path]:
        video_path = self.args.replay_video
        tracks_path = self.args.replay_tracks
        session = self.args.replay_session

        if session:
            session = session.expanduser().resolve()
            if not session.exists():
                raise RuntimeError(f"Replay session directory {session} does not exist.")
            if video_path is None:
                video_path = self._find_session_file(session, "*.mp4")
            if tracks_path is None:
                tracks_path = self._find_session_file(session, "*_tracks.jsonl")

        if video_path is None or tracks_path is None:
            raise RuntimeError(
                "Replay mode requires both video and track inputs. "
                "Provide --replay-session or both --replay-video and --replay-tracks."
            )

        video_path = video_path.expanduser().resolve()
        tracks_path = tracks_path.expanduser().resolve()

        if not video_path.exists():
            raise RuntimeError(f"Replay video {video_path} does not exist.")
        if not tracks_path.exists():
            raise RuntimeError(f"Replay track log {tracks_path} does not exist.")
        return video_path, tracks_path

    def _find_session_file(self, session: Path, pattern: str) -> Path:
        matches = sorted(session.glob(pattern))
        if not matches:
            raise RuntimeError(f"No files matching '{pattern}' found in {session}.")
        if len(matches) > 1:
            raise RuntimeError(
                f"Multiple files matching '{pattern}' found in {session}; "
                "specify the desired file explicitly."
            )
        return matches[0]

    def _surface_from_array(self, frame) -> pygame.Surface:
        if cv2 is None:
            raise RuntimeError(
                "Replay mode requires opencv-python. Install with "
                "`python3 -m pip install opencv-python`."
            )
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        surface = pygame.image.frombuffer(
            frame_rgb.tobytes(), (frame_rgb.shape[1], frame_rgb.shape[0]), "RGB"
        )
        return surface.convert()

    def _shutdown(self) -> None:
        if self.camera:
            try:
                self.camera.stop()
            except Exception:
                pass
        if self._camera_initialized:
            pygame.camera.quit()
        if self.replay_source:
            self.replay_source.close()
        if self.driver:
            self.driver.stop()
        pygame.quit()

    def run(self) -> int:
        replay_mode = self._replay_enabled()

        try:
            if replay_mode:
                self._init_replay_source()
            self._init_pygame()
            if not replay_mode:
                self._init_camera()
                self._init_driver()
        except Exception as exc:
            print(f"Initialization failed: {exc}")
            self._shutdown()
            return 1

        assert self.screen is not None
        if replay_mode:
            assert self.replay_source is not None
        else:
            assert self.camera is not None
            assert self.driver is not None

        running = True
        last_frame_time = time.time()
        target_dt = 1.0 / max(self.args.refresh_hz, 1.0)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key in (
                    pygame.K_ESCAPE,
                    pygame.K_q,
                ):
                    running = False

            now = time.time()
            if now - last_frame_time < target_dt:
                time.sleep(target_dt - (now - last_frame_time))
            last_frame_time = time.time()

            if replay_mode:
                frame_array, tracks = self.replay_source.next_frame()
                if frame_array is None:
                    if self.args.replay_loop:
                        self.replay_source.reset()
                        continue
                    break
                frame_surface = self._surface_from_array(frame_array)
            else:
                frame_surface = self.camera.get_image(self._frame_surface)
                if frame_surface is None:
                    continue
                tracks = self.driver.get_tracks()

            if frame_surface.get_size() != self.display_size:
                frame_surface = pygame.transform.smoothscale(
                    frame_surface, self.display_size
                )

            if self.args.mirror_output:
                frame_surface = pygame.transform.flip(frame_surface, True, False)

            self.screen.blit(frame_surface, (0, 0))

            self._update_overtake_alert(tracks)
            overlay_tracks = self._select_tracks(tracks)
            self._debug_tracks(tracks, overlay_tracks)
            self._draw_track_arrows(self.screen, overlay_tracks)
            self._draw_overtake_warning(self.screen)

            pygame.display.flip()
            self.clock.tick(self.args.refresh_hz)

        self._shutdown()
        return 0

    def _select_tracks(self, tracks: Dict[int, RadarTrack]) -> List[RadarTrack]:
        candidates = [
            track
            for track in tracks.values()
            if track.long_dist > 0.0 and track.long_dist <= self.args.max_distance
        ]
        candidates.sort(key=lambda track: track.long_dist)

        merged: List[RadarTrack] = []
        merge_radius = max(self.args.merge_radius, 0.0)

        for track in candidates:
            merged_into_existing = False
            for idx, existing in enumerate(merged):
                separation = math.hypot(
                    track.long_dist - existing.long_dist,
                    track.lat_dist - existing.lat_dist,
                )
                if separation <= merge_radius:
                    if track.long_dist < existing.long_dist:
                        merged[idx] = track
                    merged_into_existing = True
                    break
            if not merged_into_existing:
                merged.append(track)

        merged.sort(key=lambda track: track.long_dist)
        return merged[: max(1, self.args.track_count)]

    def _draw_track_arrows(
        self, surface: pygame.Surface, tracks: List[RadarTrack]
    ) -> None:
        width = surface.get_width()
        half_fov = max(self.args.camera_fov / 2.0, 1e-3)

        for track in tracks:
            angle_rad = math.atan2(track.lat_dist, track.long_dist)
            angle_deg = math.degrees(angle_rad)
            clamped = clamp(angle_deg, -half_fov, half_fov)
            normalized = (clamped + half_fov) / (2.0 * half_fov)
            x_pos = int(round(normalized * (width - 1)))
            if self.args.mirror_output:
                x_pos = width - 1 - x_pos
            color = self._compute_marker_color(track)
            self._draw_arrow(surface, x_pos, color)
            self._draw_track_text(surface, track, x_pos)

    def _draw_arrow(
        self, surface: pygame.Surface, center_x: int, color: Tuple[int, int, int]
    ) -> None:
        top_y = ARROW_MARGIN_TOP
        chevron = self._arrow_cache.get(color)
        if chevron is None:
            chevron = self._build_chevron_surface(color)
            self._arrow_cache[color] = chevron
        surface.blit(chevron, (center_x - chevron.get_width() // 2, top_y))

    def _build_chevron_surface(self, color: Tuple[int, int, int]) -> pygame.Surface:
        width = ARROW_HALF_WIDTH * 2
        height = ARROW_HEIGHT
        surf = pygame.Surface((width, height), pygame.SRCALPHA).convert_alpha()

        outer_points = [
            (width // 2, height),
            (0, 0),
            (width, 0),
        ]
        pygame.draw.polygon(surf, (*color, 255), outer_points)

        inner_margin_x = max(min(width // 4, width // 2 - 1), 1)
        inner_top_y = min(max(CHEVRON_INNER_OFFSET, 0), height - CHEVRON_TIP_INSET - 1)
        inner_tip_y = min(max(height - CHEVRON_TIP_INSET, inner_top_y + 1), height - 1)
        inner_points = [
            (width // 2, inner_tip_y),
            (inner_margin_x, inner_top_y),
            (width - inner_margin_x, inner_top_y),
        ]
        pygame.draw.polygon(surf, (0, 0, 0, 0), inner_points)
        return surf

    def _update_overtake_alert(self, tracks: Dict[int, RadarTrack]) -> None:
        now = time.time()
        if self._overtake_alert and now > self._overtake_alert.get("expires_at", 0.0):
            self._overtake_alert = None

        threshold = max(self.args.overtake_time_threshold, 0.01)
        min_lateral = max(self.args.overtake_min_lateral, 0.0)
        min_closing = max(self.args.overtake_min_closing_kph / 3.6, 0.0)
        duration = max(self.args.overtake_arrow_duration, 0.0)

        best_candidate: Optional[dict] = None
        best_tto = float("inf")

        for track in tracks.values():
            if track.rel_speed >= -1e-3:
                continue
            closing_speed = -track.rel_speed
            if closing_speed < min_closing:
                continue
            if track.long_dist <= 0.0:
                continue
            tto = track.long_dist / closing_speed
            if tto < 0.0 or tto > threshold:
                continue
            if abs(track.lat_dist) < min_lateral:
                continue

            side = "left" if track.lat_dist >= 0.0 else "right"
            if tto < best_tto:
                best_candidate = {
                    "side": side,
                    "expires_at": now + duration,
                    "trigger_time": now,
                    "rel_speed": track.rel_speed,
                    "tto": tto,
                }
                best_tto = tto

        if best_candidate:
            self._overtake_alert = best_candidate

    def _draw_overtake_warning(self, surface: pygame.Surface) -> None:
        if not self._overtake_alert:
            return

        now = time.time()
        expires_at = self._overtake_alert.get("expires_at", 0.0)
        if now > expires_at:
            self._overtake_alert = None
            return

        side = self._overtake_alert.get("side", "left")
        render_side = side
        if self.args.mirror_output:
            render_side = "left" if side == "right" else "right"

        arrow_surface = self._get_overtake_surface(render_side)
        if arrow_surface is None:
            return

        y = OVERTAKE_ARROW_MARGIN
        if render_side == "left":
            x = OVERTAKE_ARROW_MARGIN
        else:
            x = surface.get_width() - OVERTAKE_ARROW_MARGIN - arrow_surface.get_width()

        surface.blit(arrow_surface, (x, y))

    def _get_overtake_surface(self, side: str) -> Optional[pygame.Surface]:
        surf = self._overtake_surfaces.get(side)
        if surf is None:
            surf = self._build_overtake_arrow(side)
            if surf:
                self._overtake_surfaces[side] = surf
        return surf

    def _build_overtake_arrow(self, side: str) -> pygame.Surface:
        width = OVERTAKE_ARROW_WIDTH
        height = OVERTAKE_ARROW_HEIGHT
        surf = pygame.Surface((width, height), pygame.SRCALPHA).convert_alpha()

        if side == "left":
            points = [(width, 0), (width, height), (0, height // 2)]
        else:
            points = [(0, 0), (width, height // 2), (0, height)]

        pygame.draw.polygon(
            surf,
            (*OVERTAKE_ARROW_COLOR, OVERTAKE_ARROW_ALPHA),
            points,
        )
        return surf

    def _compute_marker_color(self, track: RadarTrack) -> Tuple[int, int, int]:
        delta_kph = abs(track.rel_speed) * 3.6
        yellow_thr = max(self.args.warn_yellow_kph, 0.0)
        red_thr = max(self.args.warn_red_kph, yellow_thr)

        if delta_kph <= yellow_thr:
            return MARKER_COLOR_GREEN
        if delta_kph >= red_thr:
            return MARKER_COLOR_RED
        return MARKER_COLOR_YELLOW

    def _draw_track_text(
        self, surface: pygame.Surface, track: RadarTrack, center_x: int
    ) -> None:
        if not self.font:
            return

        range_m = math.hypot(track.long_dist, track.lat_dist)
        range_surface = self.font.render(f"{range_m:.1f} m", True, TEXT_COLOR)
        range_rect = range_surface.get_rect()
        range_rect.midtop = (center_x, TEXT_OFFSET_TOP)
        surface.blit(range_surface, range_rect)

        speed = track.rel_speed
        # Toyota radar convention: positive rel_speed = target moving away, negative = closing.
        if speed > 0.1:
            speed_color = SPEED_COLOR_AWAY
        elif speed < -0.1:
            speed_color = SPEED_COLOR_CLOSING
        else:
            speed_color = SPEED_COLOR_STATIONARY

        speed_surface = self.font.render(
            f"{speed:+.1f} m/s ({speed * 3.6:+.1f} km/h)", True, speed_color
        )
        speed_rect = speed_surface.get_rect()
        speed_rect.midtop = (center_x, range_rect.bottom + TEXT_SPACING)
        surface.blit(speed_surface, speed_rect)

    def _debug_tracks(
        self, tracks: Dict[int, RadarTrack], overlay_tracks: List[RadarTrack]
    ) -> None:
        now = time.time()
        if now - self._last_debug_print < 0.5:
            return
        self._last_debug_print = now
        summary = ", ".join(self._format_debug_entry(track) for track in overlay_tracks)
        if not summary:
            summary = "no overlay tracks"
        print(
            f"[{time.strftime('%H:%M:%S')}] cached={len(tracks)} overlay={len(overlay_tracks)} -> {summary}"
        )

    def _format_debug_entry(self, track: RadarTrack) -> str:
        closing_speed = -track.rel_speed if track.rel_speed < 0.0 else None
        if closing_speed and track.long_dist > 0.0:
            tto = track.long_dist / closing_speed
            return (
                f"id=0x{track.track_id:02X} long={track.long_dist:.1f} lat={track.lat_dist:.1f} "
                f"rel={track.rel_speed:+.1f} tto={tto:.2f}s"
            )
        return (
            f"id=0x{track.track_id:02X} long={track.long_dist:.1f} lat={track.lat_dist:.1f} "
            f"rel={track.rel_speed:+.1f}"
        )


class ReplaySource:
    def __init__(self, video_path: Path, tracks_path: Path) -> None:
        if cv2 is None:
            raise RuntimeError(
                "Replay mode requires opencv-python. Install with "
                "`python3 -m pip install opencv-python`."
            )
        self.video_path = Path(video_path)
        self.tracks_path = Path(tracks_path)
        self.capture = cv2.VideoCapture(str(self.video_path))
        if not self.capture.isOpened():
            raise RuntimeError(f"Failed to open replay video {self.video_path}.")

        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)) or 0
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 0
        self.fps = float(self.capture.get(cv2.CAP_PROP_FPS) or 0.0)

        self._tracks_by_frame = self._load_tracks(self.tracks_path)
        self._frame_index = 0
        self._current_tracks: Dict[int, RadarTrack] = {}

    def _load_tracks(self, path: Path) -> Dict[int, List[RadarTrack]]:
        records: Dict[int, List[RadarTrack]] = {}
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                data = json.loads(line)
                frame_idx = int(data.get("frame_index", 0))
                tracks: List[RadarTrack] = []
                for entry in data.get("tracks", []):
                    tracks.append(
                        RadarTrack(
                            track_id=int(entry["track_id"]),
                            long_dist=float(entry["long_dist"]),
                            lat_dist=float(entry["lat_dist"]),
                            rel_speed=float(entry["rel_speed"]),
                            new_track=int(entry.get("new_track", 0)),
                            timestamp=float(entry.get("timestamp", 0.0)),
                            raw=entry.get("raw", {}) or {},
                        )
                    )
                records[frame_idx] = tracks
        return records

    def next_frame(self) -> Tuple[Optional[object], Dict[int, RadarTrack]]:
        if cv2 is None:
            return None, {}

        ret, frame = self.capture.read()
        if not ret:
            return None, {}

        tracks = self._tracks_by_frame.get(self._frame_index)
        if tracks is not None:
            self._current_tracks = {track.track_id: track for track in tracks}
        current_tracks = dict(self._current_tracks)
        self._frame_index += 1
        return frame, current_tracks

    def reset(self) -> None:
        if cv2 is None:
            return
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self._frame_index = 0
        self._current_tracks = {}

    def close(self) -> None:
        if cv2 is None:
            return
        if self.capture.isOpened():
            self.capture.release()


def main() -> None:
    args = parse_args()
    app = RadarCameraOverlay(args)
    sys.exit(app.run())


if __name__ == "__main__":
    main()
