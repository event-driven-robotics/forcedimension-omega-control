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
    fd.constants = types.ModuleType("forcedimension_core.constants")
    fd.constants.TIMEGUARD = 1
    for name, module in (("forcedimension_core", fd),
                         ("forcedimension_core.dhd", fd.dhd),
                         ("forcedimension_core.drd", fd.drd),
                         ("forcedimension_core.constants", fd.constants),
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


def test_redundant_public_checked_aliases_are_removed(device):
    module, _fd, _calls = device
    for name in (
        "get_serial_number_checked",
        "track_pos_checked",
        "set_tracking_params_checked",
    ):
        assert not hasattr(module.OmegaRobot, name)
    # Combined state acquisition remains a distinct recording operation.
    assert hasattr(module.OmegaRobot, "read_checked_state")


def test_timeguard_position_status_is_success_for_get_pos_and_snapshot(device):
    """Real SDK position read may return cached TimeGuard data with status 1."""
    module, fd, _calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)

    def position(out):
        out[:] = [0.012, 0.034, 0.056]
        return fd.constants.TIMEGUARD

    def velocity(out):
        out[:] = [0.0, 0.1, 0.2]
        return 0

    def force(out):
        out[:] = [0.0, 0.0, 0.3]
        return 0

    fd.dhd.getPosition = position
    fd.dhd.getLinearVelocity = velocity
    fd.dhd.getForce = force
    assert robot.get_pos() == [0.012, 0.034, 0.056]
    assert robot.read_checked_state() == (
        (0.012, 0.034, 0.056),
        (0.0, 0.1, 0.2),
        (0.0, 0.0, 0.3),
    )


@pytest.mark.parametrize("bad_status", [-1, 2, None, True])
def test_position_rejects_undocumented_or_invalid_status(device, bad_status):
    module, fd, _calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.pos = [5.0, 6.0, 7.0]

    def bad_read(out):
        out[:] = [1.0, 2.0, 3.0]
        return bad_status

    fd.dhd.getPosition = bad_read
    with pytest.raises(RuntimeError, match="DHD position read failed"):
        robot.get_pos()
    assert robot.pos == [5.0, 6.0, 7.0]


def test_timeguard_position_with_missing_values_is_rejected(device):
    module, fd, _calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.pos = [5.0, 6.0, 7.0]
    fd.dhd.getPosition = lambda out: fd.constants.TIMEGUARD
    with pytest.raises(RuntimeError, match="non-finite"):
        robot.get_pos()
    assert robot.pos == [5.0, 6.0, 7.0]


@pytest.mark.parametrize(("reader_name", "method_name"), [
    ("getLinearVelocity", "get_vel"),
    ("getForce", "get_force"),
])
def test_timeguard_remains_error_for_reads_that_only_document_zero(
    device, reader_name, method_name,
):
    module, fd, _calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)

    def read(out):
        out[:] = [1.0, 2.0, 3.0]
        return fd.constants.TIMEGUARD

    setattr(fd.dhd, reader_name, read)
    with pytest.raises(RuntimeError, match="read failed with status 1"):
        getattr(robot, method_name)()


def test_snapshot_failure_does_not_publish_partially_updated_state(device):
    module, fd, _calls = device
    robot = module.OmegaRobot(auto_open=False, auto_initialize=False)
    robot.pos = [7.0, 8.0, 9.0]
    robot.vel = [0.0, 0.0, 0.0]
    robot.force = [3.0, 2.0, 1.0]

    def position(out):
        out[:] = [0.1, 0.2, 0.3]
        return fd.constants.TIMEGUARD

    def velocity(out):
        out[:] = [0.4, 0.5, 0.6]
        return -1

    fd.dhd.getPosition = position
    fd.dhd.getLinearVelocity = velocity
    with pytest.raises(RuntimeError, match="velocity read failed"):
        robot.read_checked_state()
    assert robot.pos == [7.0, 8.0, 9.0]
    assert robot.vel == [0.0, 0.0, 0.0]
    assert robot.force == [3.0, 2.0, 1.0]
