"""Offline tests: no vendor SDK, hardware or network needed."""
from __future__ import annotations

import io
import json
import os
import tarfile
from pathlib import Path

import pytest
from omega_control import cli, doctor, sdk_installer
from omega_control import _sdk_config as sdk


@pytest.fixture
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    monkeypatch.delenv("FDSDK", raising=False)
    monkeypatch.delenv("FORCEDIM_SDK", raising=False)
    monkeypatch.setattr(sdk, "find_sdk_root", sdk.find_sdk_root)
    return tmp_path


@pytest.mark.parametrize(("system", "arch", "suffix"), [
    ("Linux", "x86_64", "linux-x86_64-gcc.tar.gz"),
    ("Linux", "arm64", "linux-aarch64-gcc.tar.gz"),
    ("Linux", "armv7l", "linux-armv7-gcc.tar.gz"),
    ("Darwin", "arm64", "macosx26.4-arm64-clang.dmg"),
    ("Darwin", "x86_64", "macosx26.4-x86_64-clang.dmg"),
    ("Windows", "AMD64", "win-x64.exe"),
    ("Windows", "x86", "win-x86.exe"),
])
def test_sdk_asset_selection(system, arch, suffix):
    assert sdk_installer.sdk_asset(system, arch).endswith(suffix)


def test_unsupported_platform():
    with pytest.raises(sdk_installer.SDKSetupError):
        sdk_installer.sdk_asset("FreeBSD", "x86_64")


def test_doctor_missing_sdk(isolated, monkeypatch, capsys):
    monkeypatch.setattr(sdk, "find_sdk_root", lambda: None)
    assert doctor.main() == 1
    assert "omega-control setup-sdk" in capsys.readouterr().out


def test_import_without_sdk(isolated, monkeypatch):
    monkeypatch.setattr(sdk, "find_sdk_root", lambda: None)
    import omega_control
    assert omega_control.PIDController
    with pytest.raises(sdk.SDKNotFoundError, match="omega-control setup-sdk"):
        omega_control.OmegaRobot


def test_register_existing_sdk(isolated):
    root = isolated / "vendor"
    (root / "lib").mkdir(parents=True)
    (root / "include").mkdir()
    assert sdk.save_sdk_root(root) == root.resolve()
    assert json.loads(sdk.config_path().read_text())["sdk_root"] == str(root)
    assert sdk.find_sdk_root() == root.resolve()
    assert sdk.configure_forcedimension_sdk() == root.resolve()


def make_archive(path, bad=None, link=None):
    with tarfile.open(path, "w:gz") as archive:
        for name in ("sdk-3.17.7/include/dhd.h", "sdk-3.17.7/lib/release/linux-x86_64-gcc/libdhd.so"):
            data = b"test"
            member = tarfile.TarInfo(name)
            member.size = len(data)
            archive.addfile(member, io.BytesIO(data))
        if bad:
            member = tarfile.TarInfo(bad)
            member.size = 3
            archive.addfile(member, io.BytesIO(b"bad"))
        if link:
            member = tarfile.TarInfo("sdk-3.17.7/lib/release/linux-x86_64-gcc/libdhd.so.1")
            member.type = tarfile.SYMTYPE
            member.linkname = link
            archive.addfile(member)


def test_extract_linux_archive(isolated):
    archive = isolated / "sdk.tar.gz"
    make_archive(archive, link="libdhd.so")
    root = sdk_installer.extract_linux(archive, isolated / "extracted")
    assert sdk.valid_sdk_root(root)
    assert (root / "lib/release/linux-x86_64-gcc/libdhd.so.1").is_symlink()


@pytest.mark.parametrize("bad", ["../escape", "/tmp/escape", "sdk-3.17.7/../../escape"])
def test_reject_archive_traversal(isolated, bad):
    archive = isolated / "sdk.tar.gz"
    make_archive(archive, bad=bad)
    with pytest.raises(sdk_installer.SDKSetupError):
        sdk_installer.extract_linux(archive, isolated / "extract")


def test_reject_symlink_escape(isolated):
    archive = isolated / "sdk.tar.gz"
    make_archive(archive, link="../../../../../../etc/passwd")
    with pytest.raises(sdk_installer.SDKSetupError):
        sdk_installer.extract_linux(archive, isolated / "extract")


def test_linux_setup_offline(isolated, monkeypatch):
    archive = isolated / "sdk.tar.gz"
    make_archive(archive)
    monkeypatch.setattr(sdk_installer.platform, "system", lambda: "Linux")
    monkeypatch.setattr(sdk_installer.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(sdk_installer, "download", lambda url, dst: dst.write_bytes(archive.read_bytes()))
    assert sdk_installer.setup_sdk(yes=True) == 0
    assert sdk.find_sdk_root() == isolated / "SDKs/ForceDimension/sdk-3.17.7"


def test_cli_invalid_sdk_path(isolated, capsys):
    assert cli.main(["setup-sdk", "--sdk-path", str(isolated / "missing")]) == 1
    assert "Not a vendor SDK root" in capsys.readouterr().err
