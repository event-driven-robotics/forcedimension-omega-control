# Force Dimension Omega Control
*Python control library for Force Dimension omega.x haptic devices.*

A Python library providing high-level control of Force Dimension omega.x haptic devices, including device initialization, motion and trajectory control, force preloading, and real-time data recording.

> **Unofficial, community-maintained project.** Built on the Force Dimension SDK; not affiliated with or endorsed by Force Dimension.

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

- macOS (ARM or Intel), Linux, or Windows with Python 3.10 to 3.12 (inclusive).
- `forcedimension_core` currently supports Python 3.10 to 3.12 only.
- A virtual environment named `omega` or another environment manager of your choice.
- Force Dimension SDK downloaded and extracted, for example `~/SDKs/ForceDimension/sdk-3.17.6`.

If you don't already have the SDK, get it from Force Dimension (https://www.forcedimension.com/software/sdk) and place it under `~/SDKs/ForceDimension/` or another location you control.

## Installation

The distribution is not yet published on PyPI. Install directly from GitHub or a local checkout.

### Install from GitHub

```bash
pip install git+https://github.com/event-driven-robotics/forcedimension-omega-control.git
```

### Install from source

```bash
git clone https://github.com/event-driven-robotics/forcedimension-omega-control.git
cd forcedimension-omega-control

pip install -e .
```

### Using Poetry in an existing project

If your application already uses [Poetry](https://python-poetry.org/), add this library
directly from GitHub (it is not yet available on PyPI):

```bash
# Run from the root of your Poetry-managed application (Python 3.10--3.12)
poetry add "git+https://github.com/event-driven-robotics/forcedimension-omega-control.git"

# Run the SDK diagnostic inside the Poetry environment
poetry run omega-control-doctor
```

Use the same Python API as other installations: `from omega_control import OmegaRobot`.
Poetry installs the Python dependency; **the vendor Force Dimension SDK must still be
installed separately** and discoverable through `FDSDK` or `FORCEDIM_SDK`
(see [SDK configuration](#configure-the-sdk-environment-variables)).

This library currently uses **setuptools** as its build backend. You do not need to
convert it to a Poetry-built package to use it *from* a Poetry-managed project.


### Install dependencies manually (legacy)

The following installs dependencies only, not this library:

```bash
python3 -m venv omega
source omega/bin/activate
python -m pip install --upgrade pip
python -m pip install forcedimension-core h5py numpy matplotlib
```

### Configure the SDK environment variables

**Important:** The package auto-detects the SDK root in common locations and accepts both
`FDSDK` and `FORCEDIM_SDK`. If your SDK is installed somewhere unusual, point either variable
at the SDK root before importing the package. The `omega-control-doctor` command below can
confirm what the package sees after installation.

#### Quick check

```bash
omega-control-doctor
```

#### Helper script for `workon`

If you use `virtualenvwrapper`, source the helper after `workon omega` instead of exporting
variables by hand each time:

```bash
source /path/to/forcedimension-omega-control/omega_control/scripts/omega_env.sh "$HOME/SDKs/ForceDimension/sdk-3.17.6"
```

You can also add that line to your virtualenv's `postactivate` hook and `omega_env_deactivate`
to the matching `postdeactivate` hook if you want the setup and cleanup to happen automatically.

#### Linux

For Linux systems, set the SDK root and, if needed, the native library path inside the active
shell session for `omega`:

```bash
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6"
export FDSDK="$FORCEDIM_SDK"
export LD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/linux-x86_64-gcc"
```

If you use a shell activation script, you can add the same exports there so they load whenever
`omega` is activated.

If import errors persist, run `omega-control-doctor` and verify that the reported SDK root matches
the one on disk.

Finally, ensure you can find the Omega robot using:
```
lsusb
```
The output should look like this:
```
Bus 001 Device 006: ID 1451:0402 Force Dimension omega.x haptic device
```
1451: Vendor
Vendor: Product

Install the udev rule with the helper script included in this repository:

```bash
sudo ./omega_control/scripts/install_udev_rules.sh
```

This installs `omega_control/udev/99-forcedimension.rules` into
`/etc/udev/rules.d/`, reloads udev rules, and triggers the update.

If you prefer manual installation, create:
```
sudo nano /etc/udev/rules.d/99-forcedimension.rules
```
and add:
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1451", MODE="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="1451", MODE="0666"
```

Then reload rules (if you used the manual path):

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### macOS Configuration

The Force Dimension SDK provides native libraries that must be discoverable by the
dynamic loader. Set the environment variables in the active `omega` shell session or in
that environment's activation script so they are restored automatically.

Adjust `FORCEDIM_SDK` to the SDK path you have on disk. For example:

```bash
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6"
export FDSDK="$FORCEDIM_SDK"
export DYLD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/mac-arm64-clang"
```

Notes:
- On macOS, `DYLD_LIBRARY_PATH` is used by the dynamic loader. Recent macOS versions
  can restrict use of DYLD variables for system processes; however, for user-space
  Python launched from your terminal this approach usually works.
- If you prefer not to modify environment variables globally, you can export them in
  the shell before running your script:

```bash
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.6"
export FDSDK="$FORCEDIM_SDK"
export DYLD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/mac-arm64-clang"
python your_script.py
```

If you are on Intel macOS, the SDK may use the `mac-x86_64-clang` library directory instead.

#### Windows

On Windows, activate your `omega` environment and set `FDSDK` to the SDK root after installing
the Force Dimension SDK. The package will try to resolve the SDK root from that environment
variable, and `omega-control-doctor` can confirm what it sees.

```powershell
$env:Path = "C:\SDKs\ForceDimension\sdk-3.17.6\bin;$env:Path"
$env:FDSDK = "C:\SDKs\ForceDimension\sdk-3.17.6"
python -m pip install git+https://github.com/event-driven-robotics/forcedimension-omega-control.git
omega-control-doctor
```

If the import still fails on Windows, check that the SDK's native DLLs are available to your
shell session before launching Python.

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

Apply controlled force along an axis using a PID outer position loop:

```python
from omega_control import OmegaRobot

robot = OmegaRobot(verbose=True)
robot.to_origin()

# Move to starting position
robot.set_pos([0.0, 0.0, -0.052])

robot.start_recording(freq=1000)

# Apply 1N downward force along Z-axis with a PID outer loop
result = robot.preload(
    target_axis="z",
    target_force=-1.0,
    kp=1e-3,
    ki=0.0,
    max_force=2.0,
    max_displacement=0.01,
)
print(result.status.value, result.final_force, result.final_position)

robot.stop_recording()
robot.save_log(filename="preload_data")
robot.close()
```

### Running the Examples

To run any example:

```bash
python omega_control/scripts/motion_example.py
python omega_control/scripts/trajectory_example.py
python omega_control/scripts/data_recording_example.py
python omega_control/scripts/preload_example.py
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
  - Verify `FDSDK` or `FORCEDIM_SDK` points to the SDK root and `LD_LIBRARY_PATH`/
    `DYLD_LIBRARY_PATH` points to the correct library directory for your platform
    (the `lib/release/<platform>` subfolder).
  - On Windows, verify `FDSDK` points to the SDK root and that the SDK DLLs are visible to the
    current shell session.
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

This project is distributed under the GNU General Public License v3.0 ([GPL-3.0](LICENSE)).
