Python Framework to control Omega3 robot. 
## omega_control — Python framework for Force Dimension Omega3

This directory contains utilities and example code to control the Force Dimension
Omega3 haptic device from Python. The code depends on the vendor SDK (ForceDimension)
and the Python package `forcedimension-core` which wraps the SDK.

This README explains how to prepare a reproducible Python environment, configure the
SDK library path on macOS and Linux, install the Python package, and run a quick smoke test.

## Prerequisites

- macOS (ARM or Intel) or Linux with a supported Python 3.12 installation (we recommend using
  Conda to manage the environment).
- Force Dimension SDK downloaded and extracted (example path: `~/SDKs/ForceDimension/sdk-3.17.6`).
- A Conda distribution (miniconda or anaconda) or another virtual environment manager.

If you don't already have the SDK, get it from Force Dimension (https://www.forcedimension.com/software/sdk) and place it under `~/SDKs/ForceDimension/` or another location you control.

## Create and activate the Conda environment

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

## Configure the SDK for this Conda environment (macOS)

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

## Quick test (smoke test)

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

## Usage

- See the Python modules under this repository for examples on how to initialise and
  command the Omega3 device. Typical steps are: initialise the library, open a
  connection to the device, then send force/position commands according to your
  control loop.

## Troubleshooting

- "Loader error" or `OSError` when importing:
  - Verify `FORCEDIM_SDK` points to the SDK root and `DYLD_LIBRARY_PATH` to the correct
    library directory for your platform (the `lib/release/<platform>` subfolder).
  - Ensure you installed the version of `forcedimension-core` compatible with your SDK.

- Device count is 0:
  - Check USB connectivity and device power.
  - Confirm the device is supported by the SDK version you downloaded.

## Contributing and contact

If you add example scripts, please keep them under the `omega_control/` folder and
document usage in this README. For questions, open an issue in the repository.

## License

This repository follows the license specified at the project root. If no license is
present, assume the code is not open-source until one is added.
