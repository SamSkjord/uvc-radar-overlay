# Radar Camera Overlay

[![Python Version](https://img.shields.io/badge/python-3.7%2B-blue)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi-red)](https://www.raspberrypi.org/)
[![AI Slop](https://img.shields.io/badge/AI%20Slop%20-%20ChatGPT%20Codex%205-beige)](https://www.morningstar.com/news/marketwatch/20251003175/the-ai-bubble-is-17-times-the-size-of-the-dot-com-frenzy-and-four-times-subprime-this-analyst-argues)

Pygame overlay that superimposes millimeter-wave radar tracks on top of a live UVC camera feed. Supports both **Toyota** (Denso, dual CAN) and **Tesla** (Bosch MRRevo14F, single CAN) radars - select with `--radar-type`. The overlay provides quick spatial context for the nearest objects detected by the radar.

## Supported Radars

| Aspect | Toyota (Denso) | Tesla (Bosch MRRevo14F) |
|--------|---------------|------------------------|
| Compatible units | Prius/Corolla 2017+ | Any Model S/X/3 unit |
| CAN buses | 2 (radar data + car keepalive) | 1 (single bus, TX+RX) |
| Track count | 16 | 32 |
| Track data | 5 fields (dist, lat, speed, new, valid) | 16+ fields (+ classification, probability) |
| Keepalive | ACC_CONTROL + 9 static frames | ~30 vehicle state messages at 100 Hz |
| VIN | Not needed | Auto-read via UDS at startup |
| DBC files | `toyota_prius_2017_adas.dbc` | `tesla_radar.dbc` + `tesla_can.dbc` |

## Features
- UVC camera capture via `pygame.camera` with fullscreen or windowed presentation.
- FOV-aware projection of radar tracks into screen space, rendered as downward chevrons that include range readouts and color-coded speed labels.
- Automatic clustering of nearby tracks to reduce duplicate markers and configurable mirroring for driver/passenger side cameras.
- Configurable speed thresholds that match Bosch CAS-M conventions for green/yellow/red status cues.
- Persistent blue overtake arrows that highlight the side a closing vehicle will pass, even if the track briefly drops out.
- Tunable CLI parameters for CAN channels, camera device/resolution/FOV, number of tracks to display, distance filtering, and warning behaviour.

## Requirements
- Python 3.8+
- `python-can`, `cantools`, and `pygame`
- Linux host with SocketCAN support (e.g., Raspberry Pi) and radar hardware connected to the CAN bus(es).
- UVC-compatible camera accessible at `/dev/video*`.

Install dependencies:

```bash
python3 -m pip install pygame python-can cantools opencv-python
```

## Running the Overlay

### Toyota Radar (default)

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

### Tesla Radar

1. Bring up the CAN interface (single bus):

   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set can0 up
   ```

2. Launch with `--radar-type tesla`:

   ```bash
   sudo python3 radar_pygame.py --radar-type tesla --windowed --camera-device /dev/video0 --camera-fov 70
   ```

   The Tesla driver auto-reads the VIN from the radar via UDS at startup. Override with `--tesla-vin <VIN>` or disable with `--no-tesla-auto-vin`.

### Common Options

   - Toggle fullscreen by omitting `--windowed`.
   - Mirroring is enabled by default; disable with `--no-mirror-output` if your camera is already mirrored.
   - Press `Esc` or `q` to exit.

### Replaying Recorded Sessions

The overlay can operate on previously captured drives recorded with `radar_capture.py`.

```bash
python3 radar_pygame.py --replay-session captures/freeway_run --replay-loop
```

- Provide `--replay-session <dir>` to autodetect the MP4 and JSONL files in a capture directory, or supply `--replay-video` / `--replay-tracks` explicitly.
- All existing overlay features (mirroring, speed thresholds, overtake detection) run as if the radar were live.
- Use `--replay-loop` to cycle continuously while tuning thresholds.

### Key CLI Flags
- `--radar-type`: `toyota` (default) or `tesla` - selects the radar driver.
- `--radar-channel` / `--car-channel`: SocketCAN interface names (Toyota).
- `--tesla-channel`: SocketCAN interface for Tesla (single bus, default `can0`).
- `--tesla-dbc`: DBC file for Tesla radar (default `opendbc/tesla_radar.dbc`).
- `--tesla-vin`: Tesla VIN override (default: auto-read via UDS).
- `--no-tesla-auto-vin`: disable Tesla VIN auto-read at startup.
- `--camera-width` / `--camera-height`: capture resolution (defaults 1280×720).
- `--camera-fov`: horizontal camera field of view in degrees used for track projection.
- `--track-count`: number of nearest radar tracks to overlay (default 3).
- `--merge-radius`: meters for deduplicating nearby radar targets (default 1.0).
- `--warn-yellow-kph` / `--warn-red-kph`: relative-speed thresholds for chevron colour changes (defaults 10 / 20 km/h).
- `--overtake-time-threshold`: time-to-overtake trigger in seconds (default 1.0).
- `--overtake-min-closing-kph`: minimum closing speed of the trailing object in km/h (default 5.0).
- `--overtake-arrow-duration`: keep the blue overtake arrow visible this many seconds after a track disappears (default 1.0).
- `--mirror-output` / `--no-mirror-output`: flip the camera feed and overlays horizontally (enabled by default).
- `--replay-session`, `--replay-video`, `--replay-tracks`, `--replay-loop`: run the overlay on captured data instead of live input.

Run `python3 radar_pygame.py --help` to view all options.

### Overtake Warning

Overtake alerts are raised when a trailing vehicle’s predicted time-to-overtake is below the configured threshold, its lateral offset exceeds `--overtake-min-lateral`, and its closing speed meets `--overtake-min-closing-kph`. When triggered, a blue chevron appears on the side of the screen corresponding to the passing direction and persists for `--overtake-arrow-duration` seconds even if the track temporarily drops.

## Repository Layout
- `toyota_radar_driver.py` - Toyota (Denso) radar driver with callback API and keep-alive management.
- `tesla_radar_driver.py` - Tesla (Bosch MRRevo14F) radar driver with 100Hz vehicle simulation and UDS VIN read.
- `tesla_radar_protocol.py` - Tesla radar CAN protocol (threaded 100Hz TX loop).
- `uds_can.py` - ISO-TP / UDS transport layer for Tesla VIN auto-read.
- `radar_pygame.py` - pygame-based camera overlay (supports both radars via `--radar-type`).
- `radar_capture.py` - records synchronized MP4 video and JSONL radar track logs for offline replay.
- `radar_curses.py` - terminal visualization using curses.
- `radar_callbacks.py` - minimal logging example.
- `opendbc/` - DBC files for Toyota and Tesla radar decoding.

## Calibration Tips
- Measure the true camera FOV and pass it via `--camera-fov` for accurate alignment.
- Adjust `--max-distance` to focus on the relevant forward range.
- Use `--track-timeout` to control how long stale tracks persist in the overlay.

## License

MIT. See [LICENSE](LICENSE) for details.
