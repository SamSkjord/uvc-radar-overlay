#!/usr/bin/env python3
"""
Pygame overlay that composites Toyota radar tracks on top of a UVC camera feed.
The overlay highlights the nearest tracks within the configured camera field of
view by drawing green arrows along the top edge of the display.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, Iterable, List

import pygame
import pygame.camera

from toyota_radar_driver import RadarTrack, ToyotaRadarConfig, ToyotaRadarDriver

# Default overlay styling
ARROW_COLOR = (0, 255, 0)
ARROW_HEIGHT = 40
ARROW_HALF_WIDTH = 18
ARROW_MARGIN_TOP = 12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Overlay Toyota radar tracks onto a UVC camera feed using pygame."
    )
    # CAN / radar configuration
    parser.add_argument("--radar-channel", default="can1", help="Radar CAN channel.")
    parser.add_argument("--car-channel", default="can0", help="Car CAN channel.")
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

    def _shutdown(self) -> None:
        if self.camera:
            try:
                self.camera.stop()
            except Exception:
                pass
        pygame.camera.quit()
        if self.driver:
            self.driver.stop()
        pygame.quit()

    def run(self) -> int:
        try:
            self._init_pygame()
            self._init_camera()
            self._init_driver()
        except Exception as exc:
            print(f"Initialization failed: {exc}")
            self._shutdown()
            return 1

        assert self.screen is not None
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

            frame = self.camera.get_image(self._frame_surface)
            if frame is None:
                continue

            if frame.get_size() != self.display_size:
                frame = pygame.transform.smoothscale(frame, self.display_size)

            self.screen.blit(frame, (0, 0))

            tracks = self.driver.get_tracks()
            overlay_tracks = self._select_tracks(tracks)
            self._draw_track_arrows(self.screen, overlay_tracks)

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
        return candidates[: max(1, self.args.track_count)]

    def _draw_track_arrows(
        self, surface: pygame.Surface, tracks: Iterable[RadarTrack]
    ) -> None:
        width = surface.get_width()
        half_fov = max(self.args.camera_fov / 2.0, 1e-3)

        for track in tracks:
            angle_rad = math.atan2(track.lat_dist, track.long_dist)
            angle_deg = math.degrees(angle_rad)
            clamped = clamp(angle_deg, -half_fov, half_fov)
            normalized = (clamped + half_fov) / (2.0 * half_fov)
            x_pos = int(round(normalized * (width - 1)))
            self._draw_arrow(surface, x_pos)

    def _draw_arrow(self, surface: pygame.Surface, center_x: int) -> None:
        top_y = ARROW_MARGIN_TOP
        points = [
            (center_x, top_y),
            (center_x - ARROW_HALF_WIDTH, top_y + ARROW_HEIGHT),
            (center_x + ARROW_HALF_WIDTH, top_y + ARROW_HEIGHT),
        ]
        pygame.draw.polygon(surface, ARROW_COLOR, points)


def main() -> None:
    args = parse_args()
    app = RadarCameraOverlay(args)
    sys.exit(app.run())


if __name__ == "__main__":
    main()
