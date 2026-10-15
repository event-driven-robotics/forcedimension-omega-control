"""SDK discovery diagnostics; safe when no SDK or robot is installed."""
from __future__ import annotations
import os
import platform
from ._sdk_config import SETUP_HINT, configure_forcedimension_sdk, library_subdir

def main() -> int:
    print("omega-control SDK doctor")
    print(f"platform: {platform.system()} {platform.release()} ({platform.machine()})")
    print(f"FDSDK: {os.environ.get('FDSDK', '<unset>')}")
    print(f"FORCEDIM_SDK: {os.environ.get('FORCEDIM_SDK', '<unset>')}")
    try:
        root = configure_forcedimension_sdk()
    except RuntimeError:
        print("status: SDK not installed or not discoverable")
        print(SETUP_HINT)
        return 1
    print(f"sdk_root: {root}")
    subdir = library_subdir()
    if subdir and not (root / subdir).is_dir():
        print(f"status: expected native library directory missing: {root / subdir}")
        return 1
    print("status: SDK directory detected")
    print("note: native library loading and connected hardware were not tested")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
