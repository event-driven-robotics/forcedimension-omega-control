"""Diagnostic helper for Force Dimension SDK setup."""

from __future__ import annotations

import os
import platform
from pathlib import Path

from omega_control._sdk_config import configure_forcedimension_sdk


def main() -> int:
    """Print the resolved SDK settings and a short next-step hint."""

    print("omega-control SDK doctor")
    print(f"platform: {platform.system()} {platform.release()} ({platform.machine()})")
    print(f"FDSDK: {os.environ.get('FDSDK', '<unset>')}")
    print(f"FORCEDIM_SDK: {os.environ.get('FORCEDIM_SDK', '<unset>')}")

    try:
        sdk_root = configure_forcedimension_sdk()
    except Exception as exc:
        print(f"status: not ready ({exc})")
        return 1

    print(f"status: ready")
    print(f"sdk_root: {sdk_root}")

    library_paths = [
        "lib/release/linux-x86_64-gcc",
        "lib/release/mac-arm64-clang",
        "lib/release/mac-x86_64-clang",
    ]
    print("candidate library directories:")
    for relative_path in library_paths:
        candidate_path = sdk_root / Path(relative_path)
        print(f"- {candidate_path} {'(exists)' if candidate_path.exists() else '(missing)'}")

    print("next step: if importing omega_control still fails, set FDSDK to the SDK root before launch")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())