"""SDK discovery and native binding diagnostics; no robot required."""
from __future__ import annotations

from importlib import import_module
import os
import platform

from ._sdk_config import SETUP_HINT, configure_forcedimension_sdk, library_subdir


def main() -> int:
    print("omega-control SDK doctor")
    print(f"platform: {platform.system()} {platform.release()} ({platform.machine()})")
    try:
        root = configure_forcedimension_sdk()
    except RuntimeError:
        print("status: SDK not installed or not discoverable")
        print(SETUP_HINT)
        return 1

    print(f"sdk_root: {root}")
    print(f"FDSDK: {os.environ.get('FDSDK', '<unset>')}")
    print(f"FORCEDIM_SDK: {os.environ.get('FORCEDIM_SDK', '<unset>')}")
    subdir = library_subdir()
    if subdir:
        libdir = root / subdir
        if not libdir.is_dir():
            print(f"status: expected native library directory missing: {libdir}")
            return 1
        if platform.system().lower() == "linux" and not any(libdir.glob("libdrd.so*")):
            print(f"status: libdrd not found in {libdir}")
            return 1

    try:
        # Loading the bindings and asking for the SDK version requires no device.
        dhd = import_module("forcedimension_core.dhd")
        version = dhd.getSDKVersion()
    except Exception as exc:
        print(f"status: native SDK library loading failed: {exc}")
        print("Check the SDK installation and native dependencies (e.g. libusb on Linux).")
        return 1

    print(f"SDK version: {version}")
    print("status: native SDK bindings loaded successfully")
    print("note: no physical robot or USB connection was tested")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
