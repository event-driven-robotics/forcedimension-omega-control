"""SDK discovery without importing the vendor bindings or requiring hardware."""
from __future__ import annotations

import json
import os
import platform
import tempfile
from pathlib import Path

SETUP_HINT = ("Force Dimension SDK not found. Run 'omega-control setup-sdk' "
              "or 'omega-control setup-sdk --sdk-path PATH'.")

class SDKNotFoundError(RuntimeError):
    """The separately installed vendor SDK could not be found."""

def library_subdir() -> str:
    system, machine = platform.system().lower(), platform.machine().lower()
    if system == "darwin":
        return "lib/release/mac-arm64-clang" if machine in ("arm64", "aarch64") else "lib/release/mac-x86_64-clang"
    if system == "linux":
        arch = {"amd64": "x86_64", "arm64": "aarch64", "armv7l": "armv7"}.get(machine, machine)
        return "lib/release/lin-" + arch + "-gcc"
    return ""

def config_path() -> Path:
    home = Path.home()
    system = platform.system().lower()
    if system == "windows":
        base = Path(os.environ.get("APPDATA", str(home / "AppData" / "Roaming")))
    elif system == "darwin":
        base = home / "Library" / "Application Support"
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME", str(home / ".config")))
    return base / "omega-control" / "sdk.json"

def valid_sdk_root(path: Path) -> bool:
    return path.is_dir() and (path / "lib").is_dir() and (path / "include").is_dir()

def find_sdk_root() -> Path | None:
    candidates = []
    for key in ("FDSDK", "FORCEDIM_SDK"):
        if os.environ.get(key):
            candidates.append(Path(os.environ[key]).expanduser())
    try:
        saved = json.loads(config_path().read_text(encoding="utf-8")).get("sdk_root")
        if isinstance(saved, str):
            candidates.append(Path(saved).expanduser())
    except (OSError, ValueError, TypeError):
        pass
    home = Path.home()
    parents = [home / "SDKs" / "ForceDimension", home / "ForceDimension",
               Path("/opt/ForceDimension"), Path("/usr/local/ForceDimension")]
    if platform.system().lower() == "windows":
        for key in ("ProgramFiles", "ProgramFiles(x86)"):
            if os.environ.get(key):
                base = Path(os.environ[key])
                parents += [base / "Force Dimension", base / "ForceDimension"]
    elif platform.system().lower() == "darwin":
        parents += [Path("/Applications/Force Dimension"), Path("/Applications/ForceDimension")]
    for parent in parents:
        if parent.exists():
            candidates += sorted(parent.glob("sdk-*"), reverse=True)
    for candidate in candidates:
        if valid_sdk_root(candidate):
            return candidate.resolve()
    return None

def save_sdk_root(path: Path) -> Path:
    root = path.expanduser().resolve()
    if not valid_sdk_root(root):
        raise ValueError(f"Not a vendor SDK root (expected lib/ and include/): {root}")
    config = config_path()
    config.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".sdk-", dir=config.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump({"sdk_root": str(root)}, output)
        os.replace(tmp, config)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return root

def configure_forcedimension_sdk() -> Path:
    root = find_sdk_root()
    if root is None:
        raise SDKNotFoundError(SETUP_HINT)
    os.environ["FDSDK"] = str(root)
    os.environ["FORCEDIM_SDK"] = str(root)
    return root
