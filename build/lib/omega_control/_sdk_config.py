"""Force Dimension SDK path detection and environment bootstrapping."""

from __future__ import annotations

import os
import platform
from pathlib import Path


def _default_library_subdir() -> str:
    """Return the platform-specific library subdirectory shipped with the SDK."""

    system = platform.system().lower()
    machine = platform.machine().lower()

    if system == "darwin":
        if machine in {"arm64", "aarch64"}:
            return "lib/release/mac-arm64-clang"
        return "lib/release/mac-x86_64-clang"

    if system == "windows":
        return ""

    if system == "linux":
        if machine in {"x86_64", "amd64"}:
            return "lib/release/linux-x86_64-gcc"
        if machine in {"aarch64", "arm64"}:
            return "lib/release/linux-aarch64-gcc"
        if machine.startswith("armv7") or machine == "armv7l":
            return "lib/release/linux-armv7-gcc"

    return ""


def _candidate_sdk_roots() -> list[Path]:
    """Return possible SDK root directories ordered from most to least likely."""

    candidates: list[Path] = []

    for env_var in ("FDSDK", "FORCEDIM_SDK"):
        value = os.environ.get(env_var)
        if value:
            candidates.append(Path(value).expanduser())

    home = Path.home()
    search_roots = [
        home / "SDKs" / "ForceDimension",
        home / "ForceDimension",
        Path("/opt/ForceDimension"),
        Path("/usr/local/ForceDimension"),
    ]
    for root in search_roots:
        if root.exists():
            candidates.extend(sorted(root.glob("sdk-*"), reverse=True))

    unique_candidates: list[Path] = []
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.expanduser()
        if resolved in seen:
            continue
        seen.add(resolved)
        unique_candidates.append(resolved)

    return unique_candidates


def configure_forcedimension_sdk() -> Path:
    """Populate the SDK environment variables when a usable installation is found.

    The Force Dimension Python bindings expect the SDK root to be exposed through
    ``FDSDK``. Some installation guides and user setups use ``FORCEDIM_SDK`` instead,
    so we bridge that gap here and also look in a few standard SDK install locations.
    """

    sdk_root = os.environ.get("FDSDK") or os.environ.get("FORCEDIM_SDK")

    if not sdk_root:
        for candidate in _candidate_sdk_roots():
            library_subdir = _default_library_subdir()
            if not library_subdir or (candidate / library_subdir).exists():
                sdk_root = str(candidate)
                break

    if not sdk_root:
        raise RuntimeError(
            "Force Dimension SDK not found. Set FDSDK or FORCEDIM_SDK to the SDK root, "
            "for example /home/you/SDKs/ForceDimension/sdk-3.17.1."
        )

    os.environ.setdefault("FDSDK", sdk_root)
    os.environ.setdefault("FORCEDIM_SDK", sdk_root)
    return Path(sdk_root)