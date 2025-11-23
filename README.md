# Car Jump Simulation with Mid-Air Pitch Control

A PyBullet physics simulation demonstrating a car performing a jump with an internal mass-shifting control system for mid-air pitch stabilization.

## Features

- **Car Jump Simulation**: Car accelerates up a ramp and performs a jump
- **PID Controller**: Shifts internal mass to control pitch angle during flight
- **Baseline Comparison**: Includes both controlled and uncontrolled (baseline) versions
- **Video Recording**: Automatically records simulation with telemetry overlays

## Project Structure

```
CPS_Project/
├── main.py              # Main simulation with cube control
├── main_no_cube.py      # Baseline simulation without control
├── modules/
│   ├── controllers.py   # PID controller implementation
│   ├── custom_envs.py   # Environment setup (car, ramp, cube)
│   └── camera.py        # Camera system for recording
├── utils/
│   ├── geom_utils.py    # Coordinate transformations
│   ├── monitor_info.py  # Landing detection
│   └── videowriter.py   # Video recording with overlays
└── cfg/
    └── config.yaml      # Configuration file
```

## Quick Start

1. **Install Dependencies** (see `INSTALLATION.md` for details):
   ```bash
   # Using conda (recommended)
   conda install -c conda-forge pybullet
   pip install -r requirements.txt
   ```

2. **Run Simulation**:
   ```bash
   # With cube control
   python main.py
   
   # Baseline (no control)
   python main_no_cube.py
   ```

3. **Results**: Videos are saved in `results/` directory

## Configuration

Edit `cfg/config.yaml` to adjust:
- Car speed profiles (SLOW, NORMAL, FAST, VERY_FAST)
- PID controller gains (Kp, Ki, Kd)
- Ramp angle and position
- Cube mass and limits

## Requirements

See `requirements.txt` and `INSTALLATION.md` for detailed setup instructions.

## Comparison

Run both versions to compare:
- **With Control**: `main.py` - Uses PID to stabilize pitch
- **Without Control**: `main_no_cube.py` - Natural physics (baseline)

