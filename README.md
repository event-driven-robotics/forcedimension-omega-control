# omega_control

Python framework for controlling Force Dimension OmegaX haptic devices

A comprehensive Python framework for controlling Force Dimension OmegaX haptic devices. This package provides high-level abstractions for device initialization, motion control, trajectory generation, force feedback, and real-time data recording.

## Features

- **Easy device control** - High-level Python interface to Force Dimension SDK
- **Motion control** - Point-to-point movements and smooth trajectory tracking  
- **Data recording** - High-frequency logging of position, velocity, and force data
- **Force control** - Adaptive force application and preloading capabilities
- **Visualization** - Built-in plotting for recorded data analysis
- **Safety features** - Proper device initialization and connection management

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)  
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Authors](#authors)
- [License](#license)

## Prerequisites

- macOS (ARM or Intel) or Linux with a supported Python 3.12 installation (we recommend using
  Conda to manage the environment).
- Force Dimension SDK downloaded and extracted (example path: `~/SDKs/ForceDimension/sdk-3.17.6`).
- A Conda distribution (miniconda or anaconda) or another virtual environment manager.

If you don't already have the SDK, get it from Force Dimension (https://www.forcedimension.com/software/sdk) and place it under `~/SDKs/ForceDimension/` or another location you control.

## Installation

### 1. Create and activate the Conda environment

Open a terminal and run:

```bash
conda create -n omega python=3.12 pip -y
conda activate omega
```

Upgrade pip and install the Python wrapper package from PyPI:

```bash
python -m pip install --upgrade pip
python -m pip install forcedimension-core
```

### 2. Configure the SDK environment variables

#### Linux Configuration

For Linux systems, use `LD_LIBRARY_PATH` instead of `DYLD_LIBRARY_PATH`:

```bash
conda env config vars set \
  FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6" \
  LD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/linux-x86_64-gcc"
conda deactivate
conda activate omega
```

Or export them temporarily:

```bash
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6"
export LD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/linux-x86_64-gcc"
python your_script.py
```

#### macOS Configuration

The Force Dimension SDK provides native libraries that must be discoverable by the
dynamic loader. We set environment variables attached to the Conda environment so they
are persistent whenever the environment is activated.

Adjust `FORCEDIM_SDK` to the SDK path you have on disk. For example:

```bash
conda env config vars set \
  FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6" \
  DYLD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/mac-arm64-clang"
conda deactivate
conda activate omega
```

Notes:
- On macOS, `DYLD_LIBRARY_PATH` is used by the dynamic loader. Recent macOS versions
  can restrict use of DYLD variables for system processes; however, for user-space
  Python launched from your terminal this approach usually works.
- If you prefer not to modify environment variables globally, you can export them in
  the shell before running your script:

```bash
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6"
export DYLD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/mac-arm64-clang"
python your_script.py
```

## Quick Start

Run a small Python snippet while the `omega` environment is active to verify the SDK
loads and the device is visible:

```python
import forcedimension_core.dhd as dhd

try:
    print("SDK version:", dhd.getSDKVersion())
    print("Device count:", dhd.getDeviceCount())
except OSError as e:
    print("Loader error:", e)

# If device count > 0 you can proceed to use the higher-level utilities in this repo.
```

Run this by saving it in a file (e.g. `test_sdk.py`) and running:

```bash
python test_sdk.py
```

## Usage Examples

This repository includes several example scripts demonstrating different capabilities of the OmegaX control framework. All examples use the `OmegaRobot` class which provides a high-level interface to the Force Dimension SDK.

### Basic Motion Control (`motion_example.py`)

Simple point-to-point movements along each axis with data recording:

```python
from omega_control import OmegaRobot

robot = OmegaRobot(verbose=False)
robot.to_origin()

robot.start_recording(freq=1000)   # Record at 1 kHz

# Move along each axis and return to origin
robot.set_pos([0.05, 0.0, 0.0])  # Move 5cm along X
robot.to_origin()
robot.set_pos([0.0, 0.05, 0.0])  # Move 5cm along Y
robot.to_origin()
robot.set_pos([0.0, 0.0, 0.05])  # Move 5cm along Z
robot.to_origin()

robot.stop_recording()
filename = robot.save_log(filename="axis_finding")
robot.close()

# Visualize the recorded data
robot.plot_log(filename=filename)
```

### Trajectory Control (`trajectory_example.py`)

Continuous sinusoidal motion with real-time trajectory tracking:

```python
import math
import time
from omega_control import OmegaRobot

# Parameters
amplitude = 0.03   # 3 cm amplitude
frequency = 0.25   # 0.25 Hz (4 second period)
duration = 5.0     # Run for 5 seconds

robot = OmegaRobot(verbose=False)
robot.to_origin()
robot.start_recording(freq=1000)

# Generate smooth trajectory
t0 = time.time()
while (time.time() - t0) < duration:
    t = time.time() - t0
    x = amplitude * math.sin(2 * math.pi * frequency * t)
    y = 1.5 * amplitude * math.sin(2 * math.pi * frequency * t)
    z = 0.5 * amplitude * math.sin(2 * math.pi * frequency * t)
    
    robot.set_pos([x, y, z], tracking=True)  # Continuous tracking
    time.sleep(0.001)  # ~1 kHz control loop

robot.stop_recording()
robot.save_log(filename="trajectory")
robot.close()
```

### Data Recording (`data_recording_example.py`)

High-frequency data collection of position, velocity, and force:

```python
import time
from omega_control import OmegaRobot

robot = OmegaRobot(verbose=True)
robot.to_origin()

# Record at high frequency
robot.start_recording(freq=10000)  # 10 kHz sampling
time.sleep(2)                      # Collect for 2 seconds
robot.stop_recording()

# Save and visualize data
filename = robot.save_log()
robot.close()
robot.plot_log(filename=filename)
```

### Force Preloading (`preload_example.py`)

Apply controlled force along an axis using adaptive positioning:

```python
from omega_control import OmegaRobot

robot = OmegaRobot(verbose=True)
robot.to_origin()

# Move to starting position
robot.set_pos([0.0, 0.0, -0.052])

robot.start_recording(freq=1000)

# Apply 1N downward force along Z-axis
robot.preload(target_axis="z", target_force=-1.0)

robot.stop_recording()
robot.save_log(filename="preload_data")
robot.close()
```

### Running the Examples

To run any example:

```bash
python motion_example.py
python trajectory_example.py  
python data_recording_example.py
python preload_example.py
```

Each example will:
1. Initialize the robot connection
2. Perform the specified motion/control task
3. Record position, velocity, and force data
4. Save data to HDF5 format
5. Generate plots showing the recorded data
6. Safely close the robot connection

## Troubleshooting

- "Loader error" or `OSError` when importing:
  - Verify `FORCEDIM_SDK` points to the SDK root and `DYLD_LIBRARY_PATH` to the correct
    library directory for your platform (the `lib/release/<platform>` subfolder).
  - Ensure you installed the version of `forcedimension-core` compatible with your SDK.

- Device count is 0:
  - Check USB connectivity and device power.
  - Confirm the device is supported by the SDK version you downloaded.

## Authors

**Maintainers:**
- Simon Mueller-Cleve ([@smullercleve](mailto:simon.mullercleve@iit.it))
- Quentin Halbach ([@qhalbach](mailto:halbach.quentin@iit.it))

**Institution:**  
[Istituto Italiano di Tecnologia (IIT)](https://www.iit.it/)

## Contributing

We welcome contributions! Please feel free to submit issues, feature requests, or pull requests.

For questions or support, please contact the maintainers above.

## License

This project is licensed under the terms specified in the [LICENSE](LICENSE) file. Please refer to that file for detailed licensing information.
