# Force Dimension Omega Control
*Python control library for Force Dimension omega.x haptic devices.*

A Python library providing high-level control of Force Dimension omega.x haptic devices, including device initialization, motion and trajectory control, force preloading, and real-time data recording.

> **Unofficial, community-maintained project.** Built on the Force Dimension SDK; not affiliated with or endorsed by Force Dimension.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [SDK setup](#sdk-setup-after-installing-the-python-library)
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
- Force Dimension's vendor SDK, installed via the guided setup command below or manually.

The vendor SDK is **not bundled** with this Python package. An opt-in setup command can download it directly from [Force Dimension](https://www.forcedimension.com/software/sdk).

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
poetry run omega-control doctor
```

Use the same Python API as other installations: `from omega_control import OmegaRobot`.
Poetry installs the Python dependencies; run `poetry run omega-control setup-sdk`
for guided vendor SDK installation (see below).

This library currently uses **setuptools** as its build backend. You do not need to
convert it to a Poetry-built package to use it *from* a Poetry-managed project.


### SDK setup (after installing the Python library)

The Force Dimension SDK is **third-party software**, not included in this
repository or downloaded during `pip install` / `poetry add`. The library can
be installed without a connected robot or SDK. When you first request the
hardware API without an SDK, it explains how to install one.

Run the guided SDK installer:

```bash
omega-control setup-sdk          # detects OS and CPU, asks before downloading
omega-control doctor             # checks SDK discovery; no robot required
```

For a Poetry-managed environment, use:

```bash
poetry run omega-control setup-sdk
poetry run omega-control doctor
```

For noninteractive use, `omega-control setup-sdk --yes` approves the vendor
download without prompting. **Nothing is downloaded automatically during
Python package installation or import.** The installer currently targets
the official Force Dimension SDK **3.17.7** from the
[manufacturer's download page](https://www.forcedimension.com/software/sdk).
Review the manufacturer's license terms before proceeding.

Installation behavior depends on the operating system:

- **Linux (x86_64, aarch64, armv7):** Downloads the official `.tar.gz` and
  extracts it into `~/SDKs/ForceDimension/sdk-3.17.7/`, then saves the path
  in per-user configuration. No administrator access is required to extract
  the SDK. USB/udev rules may still require `sudo` (see below).
- **macOS (Intel or Apple Silicon):** Downloads the official `.dmg` to
  `~/Downloads/` and opens it. Follow the vendor's installation steps;
  the CLI does not silently install system software.
- **Windows (x86/x64):** Downloads the official `.exe` to the Downloads
  folder and starts the vendor installer. Complete the installer (and
  any required driver/UAC prompts) yourself.

Already have a vendor SDK installed, or did the native installer put it
somewhere unusual? Register its root directory (the one containing
`lib/` and `include/`) without downloading anything:

```bash
omega-control setup-sdk --sdk-path /path/to/sdk-3.17.7
omega-control doctor
```

A registered path is remembered in user configuration (Linux:
`~/.config/omega-control/sdk.json`, macOS:
`~/Library/Application Support/omega-control/sdk.json`, Windows:
`%APPDATA%\\omega-control\\sdk.json`). The library sets `FDSDK` and
`FORCEDIM_SDK` **within its own Python process** before loading the vendor
bindings; it does not edit shell startup files or set permanent machine-wide
environment variables.

`omega-control-doctor` is retained as a backward-compatible alias for
`omega-control doctor`. Diagnostics check SDK discovery and load the native bindings to report the
SDK version, **without requiring a connected robot**. They do not test robot control. If you see
`Force Dimension SDK not found`, use `omega-control setup-sdk` or
`--sdk-path` as above. On macOS and Windows, rerun diagnostics after
completing the vendor's installer.

### Install dependencies manually (legacy)

The following installs dependencies only, not this library:

```bash
python3 -m venv omega
source omega/bin/activate
python -m pip install --upgrade pip
python -m pip install forcedimension-core h5py numpy matplotlib
```

### Configure the SDK environment variables

**Important:** After `omega-control setup-sdk`, the SDK root is remembered and
discovered automatically by the Python library. Existing `FDSDK` or
`FORCEDIM_SDK` environment variables and standard installation locations
remain supported. If an SDK is installed outside those locations, use
`omega-control setup-sdk --sdk-path PATH` to register it.

#### Quick check

```bash
omega-control doctor
```

#### Helper script for `workon`

If you use `virtualenvwrapper`, source the helper after `workon omega` instead of exporting
variables by hand each time:

```bash
source /path/to/forcedimension-omega-control/omega_control/scripts/omega_env.sh "$HOME/SDKs/ForceDimension/sdk-3.17.7"
```

You can also add that line to your virtualenv's `postactivate` hook and `omega_env_deactivate`
to the matching `postdeactivate` hook if you want the setup and cleanup to happen automatically.

#### Linux

For Linux systems, set the SDK root and, if needed, the native library path inside the active
shell session for `omega`:

```bash
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.7"
export FDSDK="$FORCEDIM_SDK"
export LD_LIBRARY_PATH="$FORCEDIM_SDK/lib/release/lin-x86_64-gcc"
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
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.7"
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
export FORCEDIM_SDK="$HOME/SDKs/ForceDimension/sdk-3.17.7"
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
$env:Path = "C:\SDKs\ForceDimension\sdk-3.17.7\bin;$env:Path"
$env:FDSDK = "C:\SDKs\ForceDimension\sdk-3.17.7"
python -m pip install git+https://github.com/event-driven-robotics/forcedimension-omega-control.git
omega-control-doctor
```

If the import still fails on Windows, check that the SDK's native DLLs are available to your
shell session before launching Python.

## Quick Start

After guided setup, run `omega-control doctor` to confirm the SDK path and load
the native libraries. To check for attached hardware, run a small Python snippet:

```python
# Accessing the high-level interface first configures SDK discovery.
from omega_control import OmegaRobot
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

The runnable scripts are in [`omega_control/scripts/`](omega_control/scripts/).
These scripts are the source of truth for example code, rather than maintaining
separate copies in this README. **All scripts interacting with hardware require
a supervised setup.** Device initialization, including the default
`OmegaRobot()` constructor, can move the device. Confirm a clear workspace,
suitable motion/force limits, and an accessible independent emergency stop.

Before running a hardware example, install the SDK and check discovery with:

```bash
omega-control doctor
```

The doctor command checks SDK loading, **not** the connected hardware or
physical safety.

**Initialization pattern:** Every example that initializes a robot requires
both `--device-index` and `--serial`. The script calls
`robot.open(device_index=INDEX, expected_serial=SERIAL)`, verifies the serial,
and only then calls `robot.initialize()`. This explicitly uses the DRD
index-opening API; it does **not** initialize a serial-only DHD connection.
Run the identity example first to identify the correct index and serial.
Enumeration indices can change between sessions. The serial verification is
what prevents proceeding with an unintended robot.

This choice follows the supervised omega.3 field finding recorded in
[`docs/R00B_OMEGA_HARDWARE_FINDINGS_20261117.md`](docs/R00B_OMEGA_HARDWARE_FINDINGS_20261117.md):
DRD-index opening was followed by an acknowledged initialization, whereas an
earlier DHD-serial opening was followed by failed DRD initialization and
cleanup. The difference is not yet proven to be the cause; hardware
qualification remains incomplete. No example silently falls back to another
device. An initialization acknowledgment does not verify physical safety.

| Script | Demonstrates | Motion considerations |
| --- | --- | --- |
| [`device_info_example.py`](omega_control/scripts/device_info_example.py) | Open a device without initializing, get its serial number and SDK device ID, verify expected identity, close | Requests no initialization or motion; verify SDK open behavior for your device |
| [`read_state_example.py`](omega_control/scripts/read_state_example.py) | Checked position, velocity, force and combined-state reads; display control frequency | Initialization can move the device |
| [`motion_example.py`](omega_control/scripts/motion_example.py) | Checked Cartesian moves, SDK acknowledgment, recording-health checks and cleanup | Commands motion; supervise and validate workspace |
| [`motion_parameters_example.py`](omega_control/scripts/motion_parameters_example.py) | Read, adjust and restore move/tracking velocity, acceleration and jerk limits | Initialization can move; limit changes do not command a trajectory |
| [`data_recording_example.py`](omega_control/scripts/data_recording_example.py) | Capture checked state, check recorder health, use `return_log()`, save HDF5 and plot | Initialization can move the device |
| [`preload_example.py`](omega_control/scripts/preload_example.py) | Bounded PID force preload and `PreloadStatus` result handling | Requires supervised known contact geometry |
| [`motion_example_periodic.py`](omega_control/scripts/motion_example_periodic.py) | Repeated linear movement | Commands repeated motion |
| [`trajectory_example.py`](omega_control/scripts/trajectory_example.py) | Sinusoidal tracking trajectory | Commands continuous motion |
| [`trajectory_circle_example.py`](omega_control/scripts/trajectory_circle_example.py) | Circular tracking trajectory | Commands continuous motion |
| [`trajectory_8_example.py`](omega_control/scripts/trajectory_8_example.py) | Figure-eight tracking trajectory | Commands continuous motion |

### Identifying a device without initializing it

```bash
python omega_control/scripts/device_info_example.py --device-index 0
python omega_control/scripts/device_info_example.py --serial 123456
python omega_control/scripts/device_info_example.py --device-index 0 --serial 123456
```

`--device-index` selects an SDK enumeration index. `--serial` alone opens
the device by hardware serial number for **identity-only inspection**; this
DHD serial-opening path should not be assumed compatible with the DRD
initialization workflow. Supplying both arguments verifies the selected index
against the expected serial. On mismatch, opening fails rather than silently
choosing another robot. The identity script does **not** initialize the device
or request movement, but SDK open behavior is device-dependent.

### Reading state and configuring motion limits

```bash
python omega_control/scripts/read_state_example.py --device-index 0 --serial 123456 --duration 5 --interval 0.2
python omega_control/scripts/motion_parameters_example.py --device-index 0 --serial 123456
```

The state example uses `get_pos()`, `get_vel()`, `get_force()`, and
`read_checked_state()`. Measured positions are in metres, linear velocities
in m/s, forces in N, and `get_control_freq()` reports kHz. A checked read
raises `RuntimeError` on rejected or invalid SDK data. Under the latest
checked API, a position read with DHD `TIMEGUARD=1` is accepted as a
documented successful read of **recently cached SDK position data**; it does
not establish that the sample was freshly acquired. Velocity and force reads
still require status `0`. The combined read acquires position, velocity and
force under one SDK lock, **not** as an atomically synchronized hardware
measurement or a freshness guarantee.

The parameter example shows both position-move and tracking limits. The
`get_pos_param()` return shape is `(vmax, amax, jerk, status)`.
`set_pos_param()` returns `True` for an acknowledged and validated change,
raising on failure. The example restores initialization-time defaults and
closes the device during cleanup.

### Motion and command return values

```bash
python omega_control/scripts/motion_example.py --device-index 0 --serial 123456
python omega_control/scripts/motion_example_periodic.py --device-index 0 --serial 123456
```

The checked `set_pos()` method returns `True` when the SDK acknowledges the
command and raises `RuntimeError` on a rejected command. It does **not**
independently guarantee physical arrival. `to_origin()` delegates to
`set_pos()` but returns `None` on success. `initialize()` and `stop()`
also return `True` on successful SDK acknowledgment, but a successful BRAKE
request is not proof of physical braking. Other legacy methods may still
return raw SDK status codes (for example, `hold()`), while `set_force()`
returns SDK status `0` on success.

### Recording and force preloading

```bash
python omega_control/scripts/data_recording_example.py --device-index 0 --serial 123456
python omega_control/scripts/preload_example.py --device-index 0 --serial 123456
```

The recording example calls `check_recording_health()`, stops the recorder
before accessing samples, and demonstrates `return_log()` as an alternative
to file output. `save_log()` stores the samples in HDF5 format, while
`plot_log()` visualizes them. The requested sampling frequency is **not**
a verified deterministic acquisition rate: logged time is wall-clock-derived
elapsed time rather than calibrated MCU or host-monotonic sample time.

The preload method returns a `PreloadResult`; a returned object does not
mean force acquisition succeeded. Check `result.status` (or
`result.reached`) to distinguish settled acquisition from timeout, limits,
invalid measurements, or device errors. Always configure force direction and
limits for your end-effector and contact geometry.

The trajectory examples also use the verified staged initialization workflow:

```bash
python omega_control/scripts/trajectory_example.py --device-index 0 --serial 123456
python omega_control/scripts/trajectory_circle_example.py --device-index 0 --serial 123456
python omega_control/scripts/trajectory_8_example.py --device-index 0 --serial 123456
```

Replace `0` and `123456` with your actual enumerated device index and
verified hardware serial number. **Do not run an initializing example with
guessed device identity.** All updated examples use `try/finally` to attempt
cleanup after failures.
The SDK and software cannot independently confirm an emergency stop or
physical braking. Do not infer hardware qualification from offline tests.

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
