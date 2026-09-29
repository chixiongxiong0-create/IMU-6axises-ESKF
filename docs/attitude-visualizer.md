# Attitude replay visualizer

This local desktop tool displays the body attitude produced by `imu_replay`. It reads one output CSV for a single view or two output CSVs for comparison. It does not read a live sensor.

## Install and run

Use the local Windows Python installation; a virtual environment is not required:

```powershell
py -m pip install PyVista PyVistaQt PySide6
cmake --build build
./build/tools/imu_replay.exe tests/data/replay_smoke.csv build/replay_output.csv
py -m tools.attitude_viewer build/replay_output.csv
```

The executable path changes if you use a different CMake build directory. To compare two replay outputs:

```powershell
py -m tools.attitude_viewer build/replay_output.csv path/to/second_output.csv
```

## Synthetic 12-second demo

The repository includes a deterministic 485 Hz demonstration with stationary alignment,
combined roll/pitch/yaw turns, a brief linear-acceleration burst, and vibration. From
the repository root, regenerate its raw IMU input and simulated truth, run the C
estimator, then view truth (cyan) against the estimate (amber):

```powershell
py -m tools.generate_demo_imu examples/data/demo_imu_raw.csv examples/data/demo_truth.csv
./build-final-fix/tools/imu_replay.exe examples/data/demo_imu_raw.csv examples/data/demo_estimated.csv
py -m tools.attitude_viewer examples/data/demo_truth.csv examples/data/demo_estimated.csv
```

If your CMake build directory differs, substitute its `tools/imu_replay` executable.
The generator uses a fixed seed by default and writes 5,821 samples spanning 12 seconds.
The truth file is from the synthetic motion model, not an external measurement. The
`imu_replay` estimate uses its explicit uncalibrated diagnostic mode, so its health
line may show `DEGRADED` even when the attitude is drawn. This example demonstrates
visual comparison and does not establish real-world accuracy or full acceptance.

Running `py -m tools.attitude_viewer` without file arguments opens a CSV chooser. The window's **Open CSV…** button can replace the loaded file or pair. A malformed CSV is rejected with its path and line number; the viewer does not change the input file.

The required header is exactly `timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status`. Rows must have strictly increasing, nonnegative integer nanosecond timestamps, a finite nonzero quaternion, `accel_weight` in `[0, 1]`, and unsigned integer mode, health, and status values. Quaternions are normalized for display.

## Controls and interpretation

- **Play/Pause** follows a monotonic clock. At the end it pauses without looping. The scene refreshes at about 40 Hz, so it does not render every sensor row.
- Drag the timeline to seek. Playback resumes on release only if it was playing when the drag started. Speeds are 0.25×, 0.5×, 1×, 2×, and 4×.
- With two files, choose **Side by side** or **Overlay**. Both use one playback clock. **Reset camera** restores the preset NED viewpoint.
- Cyan is stream A; amber is stream B. The fixed east-west separation in Overlay is only for display and is not an estimated position.
- The visible `timestamp_ns` is the CSV's **host reference timestamp**. The CSV has no compensated physical attitude epoch or initialized field, so the viewer does not infer either.
- `No data` means a stream has no valid sample at the current time, including gaps beyond its bounded hold duration. A nonzero `status` or `health_flags` displays a prominent `DEGRADED` line while keeping its attitude visible for diagnosis. Mode, health, status, and acceleration weight are shown as recorded, without interpreting them as physical truth.

The scene-spanning NED reference axes are north, east, and down; their faint negative halves and bright positive arrowheads remain fixed as the model turns. The model and its body axes use FRD (forward, right, down). Hover over a model's hull or nose to see its input CSV filename; the reference axes and body-axis arrows do not trigger a filename tooltip. The input `q_nb` is a Hamilton scalar-first quaternion that rotates body coordinates into navigation coordinates. The body shape is a simple orientation marker.

The viewer is for visual diagnosis, not accuracy acceptance. It has no realtime serial, network, or shared-memory input, and it does not save video. The existing C `scenarios` peak gate has a known failure (14 of 15 CTest tests pass in the current baseline); this tool does not change or retune it.
