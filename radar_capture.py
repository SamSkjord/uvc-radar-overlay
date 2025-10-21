#!/usr/bin/env python3
"""
Capture radar tracks alongside a synchronized camera recording so runs can be
replayed off-vehicle. Radar snapshots are written to JSON Lines while the UVC
camera feed is encoded to disk using OpenCV.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import cv2  # type: ignore[import]

from toyota_radar_driver import RadarTrack, ToyotaRadarConfig, ToyotaRadarDriver


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record Toyota radar tracks with synchronized UVC camera video."
    )
    # Radar / CAN configuration
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

    # Camera / capture configuration
    parser.add_argument(
        "--camera-device",
        default=0,
        help="UVC camera device index or path (e.g., 0 or /dev/video0).",
    )
    parser.add_argument("--camera-width", type=int, default=1280, help="Capture width.")
    parser.add_argument(
        "--camera-height", type=int, default=720, help="Capture height."
    )
    parser.add_argument(
        "--camera-fps", type=int, default=30, help="Capture frame rate."
    )
    parser.add_argument(
        "--video-codec",
        default="mp4v",
        help="FourCC codec for the recorded video (default: mp4v).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional capture duration in seconds; omit to run until Ctrl+C.",
    )

    # Output layout
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("captures"),
        help="Directory where capture sessions are stored.",
    )
    parser.add_argument(
        "--session-name",
        default=None,
        help="Name for this capture session (default: timestamp).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow overwriting an existing session directory.",
    )

    return parser.parse_args()


def ensure_output_paths(args: argparse.Namespace) -> Dict[str, Path]:
    session = args.session_name or datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    session_dir = args.output_dir / session
    if session_dir.exists() and not args.overwrite:
        raise SystemExit(
            f"Session directory {session_dir} already exists. Use --overwrite to replace it."
        )
    session_dir.mkdir(parents=True, exist_ok=True)

    video_path = session_dir / f"{session}.mp4"
    tracks_path = session_dir / f"{session}_tracks.jsonl"
    meta_path = session_dir / f"{session}_meta.json"

    return {
        "session_dir": session_dir,
        "video_path": video_path,
        "tracks_path": tracks_path,
        "meta_path": meta_path,
    }


def serialize_track(track: RadarTrack) -> Dict[str, float]:
    return {
        "track_id": track.track_id,
        "long_dist": track.long_dist,
        "lat_dist": track.lat_dist,
        "rel_speed": track.rel_speed,
        "new_track": int(track.new_track),
        "timestamp": track.timestamp,
    }


class GracefulExit(Exception):
    pass


def install_signal_handlers() -> None:
    def handle_signal(signum, frame):  # type: ignore[override]
        raise GracefulExit()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)


def main() -> None:
    args = parse_args()
    output = ensure_output_paths(args)
    install_signal_handlers()

    try:
        config = ToyotaRadarConfig(
            radar_channel=args.radar_channel,
            car_channel=args.car_channel,
            interface=args.interface,
            radar_interface=args.radar_interface,
            car_interface=args.car_interface,
            bitrate=args.bitrate,
            radar_dbc=args.radar_dbc,
            control_dbc=args.control_dbc,
            keepalive_rate_hz=args.keepalive_rate_hz,
            track_timeout=args.track_timeout,
            notifier_timeout=args.notifier_timeout,
            auto_setup=not args.no_setup,
            use_sudo=args.use_sudo,
            setup_extra_args=args.setup_extra,
            keepalive_enabled=not args.no_keepalive,
        )

        driver = ToyotaRadarDriver(config)
        driver.start()
    except Exception as exc:
        raise SystemExit(f"Failed to start radar driver: {exc}")

    camera: Optional[cv2.VideoCapture] = None
    video_writer: Optional[cv2.VideoWriter] = None

    try:
        camera = cv2.VideoCapture(args.camera_device)
        if not camera.isOpened():
            raise SystemExit(f"Failed to open camera {args.camera_device}.")

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FPS, args.camera_fps)

        width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.camera_width
        height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.camera_height
        fps = camera.get(cv2.CAP_PROP_FPS) or float(args.camera_fps)

        fourcc = cv2.VideoWriter_fourcc(*args.video_codec)
        video_writer = cv2.VideoWriter(
            str(output["video_path"]), fourcc, fps, (width, height)
        )
        if not video_writer.isOpened():
            raise SystemExit(
                f"Failed to initialize video writer at {output['video_path']}."
            )

        metadata = {
            "created_utc": datetime.utcnow().isoformat() + "Z",
            "camera_device": args.camera_device,
            "resolution": {"width": width, "height": height},
            "fps": fps,
            "codec": args.video_codec,
            "radar_channel": args.radar_channel,
            "car_channel": args.car_channel,
            "duration_limit": args.duration,
            "track_timeout": args.track_timeout,
        }
        output["meta_path"].write_text(json.dumps(metadata, indent=2))

        flush_interval = max(1, int(round(fps)) or args.camera_fps or 1)

        with output["tracks_path"].open("w", encoding="utf-8") as track_file:
            frame_index = 0
            start_monotonic = time.monotonic()
            print(
                f"Recording capture to {output['session_dir']} "
                f"(press Ctrl+C to stop)..."
            )

            while True:
                if (
                    args.duration
                    and (time.monotonic() - start_monotonic) >= args.duration
                ):
                    print("Capture duration reached; stopping.")
                    break

                ret, frame = camera.read()
                if not ret:
                    print("Warning: Failed to read frame from camera.", file=sys.stderr)
                    time.sleep(0.05)
                    continue

                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height))

                video_writer.write(frame)

                tracks = driver.get_tracks()
                snapshot = {
                    "timestamp": time.time(),
                    "frame_index": frame_index,
                    "tracks": [serialize_track(track) for track in tracks.values()],
                }
                track_file.write(json.dumps(snapshot) + "\n")
                if frame_index % flush_interval == 0:
                    track_file.flush()
                frame_index += 1
    except GracefulExit:
        print("\nCapture interrupted; finalizing files...")
    finally:
        if camera is not None:
            camera.release()
        if video_writer is not None:
            video_writer.release()
        driver.stop()

    print(
        f"Capture complete. Video saved to {output['video_path']} and "
        f"tracks to {output['tracks_path']}."
    )


if __name__ == "__main__":
    main()
