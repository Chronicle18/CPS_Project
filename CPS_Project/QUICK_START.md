# Quick Start Guide

## Run the Simulations

### 1. With Cube Control (PID)
```bash
cd /Users/akashdudhane/Desktop/cps/CPS_Project
python main.py
```

### 2. Without Cube Control (Baseline)
```bash
python main_no_cube.py
```

## Options

**GUI Mode** (see simulation in real-time):
```bash
python main.py --mode GUI
```

**Headless Mode** (faster, no display):
```bash
python main.py --mode HEADLESS
```

**Custom output filename**:
```bash
python main.py --output my_video.mp4
```

## Results Location

Videos are saved in: `results/` folder
- `car_jump_[timestamp].mp4` - with control
- `car_jump_no_cube_[timestamp].mp4` - baseline

## If Dependencies Are Missing

Try installing with conda (recommended for pybullet):
```bash
conda install -c conda-forge pybullet
pip install imageio imageio-ffmpeg matplotlib tqdm numpy opencv-python PyYAML
```

Or use pre-built wheels:
```bash
pip install --upgrade pip
pip install pybullet --only-binary :all:
pip install imageio imageio-ffmpeg matplotlib tqdm numpy opencv-python PyYAML
```

