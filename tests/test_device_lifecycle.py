"""Hardware-free lifecycle and device identity regression tests."""
import importlib
import sys
import types
from pathlib import Path

import pytest


@pytest.fixture
def device(monkeypatch):
    package = types.ModuleType("omega_control")
    package.__path__ = [str(Path(__file__).parents[1] / "omega_control")]
    monkeypatch.setitem(sys.modules, "omega_control", package)
    config = types.ModuleType("omega_control._sdk_config")
    config.configure_forcedimension_sdk = lambda: None
    monkeypatch.setitem(sys.modules, "omega_control._sdk_config", config)
    fd = types.ModuleType("forcedimension_core")
    fd.dhd = types.ModuleType("forcedimension_core.dhd")
    fd.drd = types.ModuleType("forcedimension_core.drd")
    for name, module in (("forcedimension_core", fd),
                         ("forcedimension_core.dhd", fd.dhd),
                         ("forcedimension_core.drd", fd.drd),
                         ("h5py", types.ModuleType("h5py")),
                         ("numpy", types.ModuleType("numpy")),
                         ("matplotlib", types.ModuleType("matplotlib")),
                         ("matplotlib.pyplot", types.ModuleType("matplotlib.pyplot"))):
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules["numpy"].ndarray = object
    monkeypatch.delitem(sys.modules, "omega_control.omega_control", raising=False)
    module = importlib.import_module("omega_control.omega_control")
    calls = []
    fd.drd.open = lambda: calls.append("open") or 3
    fd.drd.openID = lambda index: calls.append(("openID", index)) or 3
    fd.dhd.openSerial = lambda serial: calls.append(("openSerial", serial)) or 3
    fd.dhd.getSerialNumber = lambda device_id: 42
    fd.drd.isInitialized = lambda device_id: False
    fd.drd.autoInit = lambda device_id: calls.append("autoInit") or 0
    fd.drd.precisionInit = lambda device_id: calls.append("precisionInit") or 0
    fd.drd.start = lambda device_id: calls.append("start") or 0
    fd.drd.getPosMoveParam = lambda device_id: (.1, .2, .3, 0)
    fd.drd.getPosTrackParam = lambda device_id: (.1, .2, .3, 0)
    fd.drd.close = lambda device_id: calls.append("close") or 0
    fd.dhd.stop = lambda device_id: calls.append("stop") or 0
    return module, fd, calls


def test_staged_construction_never_opens_or_initializes(device):
    module, fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    assert not calls and not robot.is_open()
    robot.open(device_index=1, expected_serial=42)
    assert calls == [("openID", 1)]
    assert robot.get_serial_number() == 42
    robot.initialize()
    assert calls == [("openID", 1), "autoInit"]
    assert robot.default_track_param == (.1, .2, .3, 0)


def test_legacy_constructor_initializes(device):
    module, fd, calls = device
    robot = module.OmegaRobot()
    assert calls == ["open", "autoInit"]
    assert robot.is_open()


def test_serial_only_open_and_identity_mismatch(device):
    module, fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.open(expected_serial=42)
    assert calls == [("openSerial", 42)]
    robot.close()
    fd.dhd.getSerialNumber = lambda device_id: 43
    with pytest.raises(RuntimeError, match="serial mismatch"):
        robot.open(device_index=2, expected_serial=42)
    assert calls[-2:] == [("openID", 2), "close"]
    assert not robot.is_open()


@pytest.mark.parametrize("status", [-1, None])
def test_initialization_status_failure_requests_brake_and_closes(device, status):
    module, fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.open(expected_serial=42)
    fd.drd.autoInit = lambda device_id: status
    with pytest.raises(RuntimeError, match="Initialization failed"):
        robot.initialize()
    assert calls[-2:] == ["stop", "close"]
    assert not robot.is_open()


def test_failed_cleanup_preserves_uncertain_open_state(device):
    module, fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.open()
    fd.drd.autoInit = lambda device_id: -1
    fd.drd.close = lambda device_id: -1
    with pytest.raises(RuntimeError, match="DRD close returned -1"):
        robot.initialize()
    assert robot.is_open()


def test_constructor_rejects_conflicting_auto_flags(device):
    module, fd, calls = device
    with pytest.raises(ValueError, match="requires"):
        module.OmegaRobot(auto_open=False)
    assert not calls


def test_open_rejects_invalid_selection_without_sdk_call(device):
    module, fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    with pytest.raises(ValueError):
        robot.open(device_index=-1)
    with pytest.raises(ValueError):
        robot.open(expected_serial="42")
    assert not calls


def test_original_serial_number_checks_sdk_and_preserves_return_type(device):
    module, fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.open()
    assert robot.get_serial_number() == 42

    fd.dhd.getSerialNumber = lambda _id: -1
    with pytest.raises(RuntimeError, match="getSerialNumber failed"):
        robot.get_serial_number()


def test_serial_number_requires_open_device(device):
    module, _fd, calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    with pytest.raises(RuntimeError, match="not open"):
        robot.get_serial_number()
    assert not calls
