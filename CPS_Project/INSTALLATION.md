# Installation Instructions

## Prerequisites

- Python 3.7 or higher
- pip or conda

## Installation Methods

### Method 1: Using Conda (Recommended for PyBullet)

PyBullet can be difficult to install on some systems. Conda usually works best:

```bash
# Install PyBullet via conda
conda install -c conda-forge pybullet

# Install other dependencies via pip
pip install imageio imageio-ffmpeg matplotlib tqdm numpy opencv-python PyYAML
```

### Method 2: Using pip (Standard)

```bash
pip install -r requirements.txt
```

**Note:** If PyBullet fails to install with pip (common on macOS), try:
```bash
pip install --upgrade pip
pip install pybullet --only-binary :all:
```

### Method 3: Pre-built PyBullet Wheel

If both methods fail, you can try downloading a pre-built wheel from:
- PyPI: https://pypi.org/project/pybullet/
- Or use conda (Method 1)

## Verify Installation

Test that PyBullet is installed:
```bash
python -c "import pybullet; print('PyBullet installed successfully!')"
```

## Running the Code

Once dependencies are installed:

```bash
# With cube control (PID)
python main.py

# Without cube control (baseline)
python main_no_cube.py
```

## Troubleshooting

### macOS Issues
- Use conda instead of pip for PyBullet
- Or try: `pip install pybullet --only-binary :all:`

### Linux/Windows
- Standard pip install should work: `pip install -r requirements.txt`

### If you get "ModuleNotFoundError"
- Make sure you're in the correct virtual environment
- Verify installation: `pip list | grep pybullet`

