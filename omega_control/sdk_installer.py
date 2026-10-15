"""Opt-in setup for the separately licensed Force Dimension vendor SDK."""
from __future__ import annotations

import os
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path, PurePosixPath
from urllib.parse import urlparse

from ._sdk_config import find_sdk_root, save_sdk_root, valid_sdk_root

VERSION = "3.17.7"
DOWNLOAD_PAGE = "https://www.forcedimension.com/software/sdk"
BASE_URL = "https://downloads.forcedimension.com/sdk/"
MAX_DOWNLOAD = 1_000_000_000
MAX_EXTRACTED = 2_000_000_000

class SDKSetupError(RuntimeError):
    """User-facing SDK setup error."""

def sdk_asset(system: str | None = None, machine: str | None = None) -> str:
    system = (system or platform.system()).lower()
    machine = (machine or platform.machine()).lower()
    arch = {"amd64": "x86_64", "arm64": "aarch64", "armv7l": "armv7",
            "i386": "x86", "i686": "x86"}.get(machine, machine)
    if system == "linux" and arch in ("x86_64", "aarch64", "armv7"):
        return f"sdk-{VERSION}-linux-{arch}-gcc.tar.gz"
    if system == "darwin" and arch in ("x86_64", "aarch64"):
        return f"sdk-{VERSION}-macosx26.4-{'arm64' if arch == 'aarch64' else 'x86_64'}-clang.dmg"
    if system == "windows" and arch in ("x86_64", "x86"):
        return f"sdk-{VERSION}-win-{'x64' if arch == 'x86_64' else 'x86'}.exe"
    raise SDKSetupError(f"Unsupported platform {system}/{machine}; see {DOWNLOAD_PAGE}")

def download(url: str, destination: Path) -> None:
    parsed = urlparse(url)
    if parsed.scheme != "https" or parsed.hostname != "downloads.forcedimension.com":
        raise SDKSetupError("Only official HTTPS SDK downloads are permitted")
    try:
        with urllib.request.urlopen(
            urllib.request.Request(url, headers={"User-Agent": "omega-control/1.0"}), timeout=40
        ) as response:
            final = urlparse(response.geturl())
            if final.scheme != "https" or final.hostname != "downloads.forcedimension.com":
                raise SDKSetupError("SDK download redirected away from official host")
            if response.headers.get("Content-Length") and int(response.headers["Content-Length"]) > MAX_DOWNLOAD:
                raise SDKSetupError("SDK download exceeds safety limit")
            with destination.open("wb") as output:
                total = 0
                while chunk := response.read(1024 * 1024):
                    total += len(chunk)
                    if total > MAX_DOWNLOAD:
                        raise SDKSetupError("SDK download exceeds safety limit")
                    output.write(chunk)
    except OSError as exc:
        raise SDKSetupError(f"Vendor download failed: {exc}") from exc

def safe_path(root: Path, name: str) -> Path:
    normalized = name.replace("\\", "/")
    parts = PurePosixPath(normalized)
    if parts.is_absolute() or ".." in parts.parts or ":" in normalized:
        raise SDKSetupError(f"Unsafe SDK archive entry: {name}")
    result = root.joinpath(*parts.parts)
    if not result.resolve().is_relative_to(root.resolve()):
        raise SDKSetupError(f"SDK archive entry escapes destination: {name}")
    return result

def extract_linux(archive: Path, dest: Path) -> Path:
    dest.mkdir(parents=True, exist_ok=True)
    links = []
    try:
        with tarfile.open(archive, "r:gz") as tar:
            members = tar.getmembers()
            if sum(m.size for m in members if m.isfile()) > MAX_EXTRACTED:
                raise SDKSetupError("Extracted SDK exceeds safety limit")
            for member in members:
                path = safe_path(dest, member.name)
                if member.isdir():
                    path.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    path.parent.mkdir(parents=True, exist_ok=True)
                    source = tar.extractfile(member)
                    if source is None:
                        raise SDKSetupError(f"Cannot read SDK file {member.name}")
                    with source, path.open("wb") as output:
                        shutil.copyfileobj(source, output)
                    path.chmod(member.mode & 0o777)
                elif member.issym() or member.islnk():
                    links.append((member, path))
                elif member.type not in (
                    tarfile.XHDTYPE, tarfile.XGLTYPE,
                    tarfile.GNUTYPE_LONGNAME, tarfile.GNUTYPE_LONGLINK
                ):
                    raise SDKSetupError(f"Unexpected SDK archive entry: {member.name}")
            for member, path in links:
                if member.issym():
                    if Path(member.linkname).is_absolute() or ":" in member.linkname:
                        raise SDKSetupError("Unsafe absolute symlink in SDK")
                    safe_path(dest, str(path.relative_to(dest).parent / member.linkname))
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.symlink_to(member.linkname)
                else:
                    source = safe_path(dest, member.linkname)
                    if not source.is_file():
                        raise SDKSetupError("Broken hardlink in SDK")
                    path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, path)
    except (tarfile.TarError, EOFError) as exc:
        raise SDKSetupError(f"Invalid SDK archive: {exc}") from exc
    if valid_sdk_root(dest):
        return dest
    candidates = [p for p in dest.iterdir() if p.is_dir() and valid_sdk_root(p)]
    if len(candidates) != 1:
        raise SDKSetupError("SDK archive does not contain a recognizable lib/ and include/ root")
    return candidates[0]

def setup_sdk(*, yes: bool = False, sdk_path: str | None = None) -> int:
    if sdk_path:
        print(f"Registered existing SDK: {save_sdk_root(Path(sdk_path))}")
        return 0
    existing = find_sdk_root()
    if existing:
        print(f"Existing SDK detected and registered: {save_sdk_root(existing)}")
        return 0
    asset = sdk_asset()
    url = BASE_URL + asset
    print(f"Detected {platform.system()} / {platform.machine()}")
    print(f"Official Force Dimension SDK {VERSION}: {url}")
    print(f"Review vendor terms at {DOWNLOAD_PAGE}")
    if not yes:
        if not sys.stdin.isatty():
            print("No interactive confirmation; rerun with --yes to approve download.")
            return 1
        if input("Download and set up this third-party SDK? [y/N] ").strip().lower() not in ("y", "yes"):
            print("Cancelled; no files downloaded.")
            return 1
    system = platform.system().lower()
    if system == "linux":
        parent = Path.home() / "SDKs" / "ForceDimension"
        parent.mkdir(parents=True, exist_ok=True)
        target = parent / f"sdk-{VERSION}"
        if target.exists():
            raise SDKSetupError(f"SDK destination already exists: {target}")
        with tempfile.TemporaryDirectory(prefix=".omega-sdk-", dir=parent) as tmp:
            archive = Path(tmp) / asset
            download(url, archive)
            root = extract_linux(archive, Path(tmp) / "unpacked")
            root.rename(target)
        print(f"Installed SDK: {save_sdk_root(target)}")
        print("USB udev rules may require administrator setup; see README.")
    else:
        downloads = Path.home() / "Downloads"
        downloads.mkdir(parents=True, exist_ok=True)
        destination = downloads / asset
        with tempfile.TemporaryDirectory(prefix="omega-sdk-", dir=downloads) as tmp:
            staged = Path(tmp) / asset
            download(url, staged)
            os.replace(staged, destination)
        print(f"Downloaded vendor installer: {destination}")
        if system == "windows":
            os.startfile(str(destination))
        else:
            subprocess.Popen(["open", str(destination)])
        print("Complete the vendor installer, then run 'omega-control doctor'.")
        print("If not detected, use 'omega-control setup-sdk --sdk-path PATH'.")
    return 0
