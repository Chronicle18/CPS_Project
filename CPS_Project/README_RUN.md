# How to Run the Car Jump Simulation

## Step 1: Install Dependencies

First, make sure you have all required packages installed:

```bash
cd /Users/akashdudhane/Desktop/cps/CPS_Project
pip install -r requirements.txt
```

## Step 2: Run the Simulations

### Option A: With Cube Control (PID Controller)
```bash
python main.py
```

### Option B: Without Cube Control (Baseline)
```bash
python main_no_cube.py
```

### Command Line Options

Both scripts support these options:

```bash
# Run in GUI mode (see simulation in real-time)
python main.py --mode GUI

# Run in headless mode (faster, no display)
python main.py --mode HEADLESS

# Specify custom output video filename
python main.py --output my_video.mp4

# Use custom config file
python main.py --config cfg/config.yaml
```

## Step 3: Find Your Results

After running, you'll find:

1. **Video files** in the `results/` directory:
   - `car_jump_[timestamp].mp4` - with cube control
   - `car_jump_no_cube_[timestamp].mp4` - baseline (no control)

2. **Console output** showing:
   - Speed mode and settings
   - Airborne detection messages
   - Landing detection
   - Total airtime
   - Video save confirmation

## What to Expect

### With Cube Control (`main.py`):
- Car jumps off ramp
- Cube shifts forward/backward to control pitch
- Car should land more upright/stably

### Without Cube Control (`main_no_cube.py`):
- Car jumps off ramp
- Cube stays fixed at initial position
- Car follows natural physics (baseline behavior)

## Comparing Results

1. Run both versions:
   ```bash
   python main.py --mode HEADLESS --output with_control.mp4
   python main_no_cube.py --mode HEADLESS --output no_control.mp4
   ```

2. Compare the videos in `results/` folder:
   - Look at pitch angle during flight
   - Compare landing quality
   - Check airtime duration

## Troubleshooting

- **If you get "ModuleNotFoundError"**: Run `pip install -r requirements.txt`
- **If video doesn't save**: Check that `results/` directory exists (it's created automatically)
- **If simulation is slow**: Use `--mode HEADLESS` for faster execution
- **If car doesn't move**: Check `cfg/config.yaml` speed_profile settings

