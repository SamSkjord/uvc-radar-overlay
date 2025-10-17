# Radar Camera Overlay

[![Python Version](https://img.shields.io/badge/python-3.7%2B-blue)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi-red)](https://www.raspberrypi.org/)
[![AI Slop](https://img.shields.io/badge/AI%20Slop%20-%20ChatGPT%20Codex%205-beige)](https://www.morningstar.com/news/marketwatch/20251003175/the-ai-bubble-is-17-times-the-size-of-the-dot-com-frenzy-and-four-times-subprime-this-analyst-argues)

Pygame overlay that superimposes millimeter-wave radar tracks on top of a live UVC camera feed. The project pairs the reusable radar driver from `toyota_radar_driver.py` with a lightweight visualization in `radar_pygame.py`, providing quick spatial context for the three nearest objects detected by the radar.

## Features
- UVC camera capture via `pygame.camera` with fullscreen or windowed presentation.
- FOV-aware projection of radar tracks into screen space, rendered as green arrows on the camera feed.
- Tunable CLI parameters for CAN channels, camera device/resolution/FOV, number of tracks to display, and distance filtering.

## Requirements
- Python 3.8+
- `python-can`, `cantools`, and `pygame`
- Linux host with SocketCAN support (e.g., Raspberry Pi) and Toyota radar hardware connected to `car` and `radar` CAN buses.
- UVC-compatible camera accessible at `/dev/video*`.

Install dependencies:

```bash
python3 -m pip install pygame python-can cantools
```

## Running the Overlay

1. Bring up the CAN interfaces (skip if `radar_pygame.py` is allowed to set them up):

   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set can1 type can bitrate 500000
   sudo ip link set can0 up
   sudo ip link set can1 up
   ```

2. Launch the overlay:

   ```bash
   sudo python3 radar_pygame.py --windowed --camera-device /dev/video0 --camera-fov 70
   ```

   - Toggle fullscreen by omitting `--windowed`.
   - Press `Esc` or `q` to exit.

### Key CLI Flags
- `--radar-channel` / `--car-channel`: SocketCAN interface names.
- `--camera-width` / `--camera-height`: capture resolution (defaults 1280×720).
- `--camera-fov`: horizontal camera field of view in degrees used for track projection.
- `--track-count`: number of nearest radar tracks to overlay (default 3).
- `--max-distance`: longitudinal distance cutoff in meters (default 120).

Run `python3 radar_pygame.py --help` to view all options.

## Repository Layout
- `toyota_radar_driver.py` – reusable driver with callback API and keep-alive management.
- `radar_pygame.py` – pygame-based camera overlay.
- `radar_curses.py` – terminal visualization using curses.
- `radar_callbacks.py` – minimal logging example.
- `opendbc/` – DBC files required for decoding radar messages.

## Calibration Tips
- Measure the true camera FOV and pass it via `--camera-fov` for accurate alignment.
- Adjust `--max-distance` to focus on the relevant forward range.
- Use `--track-timeout` to control how long stale tracks persist in the overlay.

## License

MIT. See [LICENSE](LICENSE) for details.
