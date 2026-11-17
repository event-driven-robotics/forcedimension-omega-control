"""Simulation tests for the preload hardware loop."""

import importlib
import sys
import types
from pathlib import Path

import pytest


@pytest.fixture
def simulated_robot(monkeypatch):
    package_path = Path(__file__).parents[1] / "omega_control"
    package = types.ModuleType("omega_control")
    package.__path__ = [str(package_path)]
    monkeypatch.setitem(sys.modules, "omega_control", package)

    sdk_config = types.ModuleType("omega_control._sdk_config")
    sdk_config.configure_forcedimension_sdk = lambda: Path("/tmp")
    monkeypatch.setitem(sys.modules, "omega_control._sdk_config", sdk_config)

    forcedimension = types.ModuleType("forcedimension_core")
    drd = types.ModuleType("forcedimension_core.drd")
    dhd = types.ModuleType("forcedimension_core.dhd")
    forcedimension.drd = drd
    forcedimension.dhd = dhd
    monkeypatch.setitem(sys.modules, "forcedimension_core", forcedimension)
    monkeypatch.setitem(sys.modules, "forcedimension_core.drd", drd)
    monkeypatch.setitem(sys.modules, "forcedimension_core.dhd", dhd)

    numpy = types.ModuleType("numpy")
    numpy.ndarray = object
    monkeypatch.setitem(sys.modules, "numpy", numpy)
    monkeypatch.setitem(sys.modules, "h5py", types.ModuleType("h5py"))
    matplotlib = types.ModuleType("matplotlib")
    pyplot = types.ModuleType("matplotlib.pyplot")
    matplotlib.pyplot = pyplot
    monkeypatch.setitem(sys.modules, "matplotlib", matplotlib)
    monkeypatch.setitem(sys.modules, "matplotlib.pyplot", pyplot)

    # Each test needs the module bound to this fixture's isolated fake SDK.
    # Otherwise importlib can reuse the previous test's cached DHD/DRD modules.
    monkeypatch.delitem(sys.modules, "omega_control.omega_control", raising=False)
    module = importlib.import_module("omega_control.omega_control")
    state = {
        "time": 0.0,
        "position": [0.0, 0.0, 0.0],
        "held": False,
    }

    class FakeTime:
        @staticmethod
        def monotonic():
            return state["time"]

        @staticmethod
        def sleep(duration):
            state["time"] += duration

    module.time = FakeTime

    def track_position(target):
        state["position"] = target.copy()
        return 0

    drd.trackPos = track_position
    robot = module.OmegaRobot.__new__(module.OmegaRobot)
    robot.verbose = False
    robot._sdk_lock = __import__("threading").RLock()
    robot.get_pos = lambda: state["position"].copy()
    robot.get_force = lambda: [100.0 * value for value in state["position"]]
    robot.hold = lambda: state.update(held=True) or 0
    return robot, module, state


def test_preload_settles_against_simulated_spring(simulated_robot):
    robot, module, state = simulated_robot

    result = robot.preload(
        "z",
        1.0,
        kp=0.05,
        control_freq=100.0,
        force_ramp_rate=10.0,
        tolerance=0.01,
        settle_time=0.05,
        timeout=2.0,
        max_velocity=0.05,
        max_displacement=0.02,
        max_force=2.0,
    )

    assert result.status is module.PreloadStatus.SETTLED
    assert result.reached
    assert abs(result.final_force - 1.0) <= 0.01
    assert result.iterations == len(result.samples)
    assert state["held"]


def test_preload_stops_at_displacement_limit(simulated_robot):
    robot, module, state = simulated_robot

    result = robot.preload(
        "z",
        1.0,
        kp=0.05,
        control_freq=100.0,
        force_ramp_rate=10.0,
        timeout=2.0,
        max_velocity=0.05,
        max_displacement=0.0001,
        max_force=2.0,
    )

    assert result.status is module.PreloadStatus.MAX_DISPLACEMENT
    assert not result.reached
    assert state["held"]


def test_checked_state_reads_three_vectors_and_rejects_sdk_errors(simulated_robot):
    robot, module, _state = simulated_robot
    robot.pos = robot.vel = robot.force = [0.0, 0.0, 0.0]

    def fill(values, content, status=0):
        values[:] = content
        return status

    module.dhd.getPosition = lambda buf: fill(buf, [0.01, 0.02, 0.03])
    module.dhd.getLinearVelocity = lambda buf: fill(buf, [0.0, 0.0, 0.0])
    module.dhd.getForce = lambda buf: fill(buf, [0.1, 0.2, 0.3])
    position, velocity, force = robot.read_checked_state()
    assert position == (0.01, 0.02, 0.03)
    assert velocity == (0.0, 0.0, 0.0)
    assert force == (0.1, 0.2, 0.3)
    assert robot.force == [0.1, 0.2, 0.3]

    module.dhd.getForce = lambda buf: fill(buf, [0.0, 0.0, 0.0], -1)
    with pytest.raises(RuntimeError, match="DHD force read failed"):
        robot.read_checked_state()
    module.dhd.getForce = lambda buf: fill(buf, [float("nan"), 0, 0])
    with pytest.raises(RuntimeError, match="non-finite"):
        robot.read_checked_state()
    module.dhd.getForce = lambda buf: fill(buf, [0, 0, 0], None)
    with pytest.raises(RuntimeError, match="DHD force read failed"):
        robot.read_checked_state()


def test_tracking_rejects_negative_status_and_invalid_targets(simulated_robot):
    robot, module, state = simulated_robot
    assert robot.set_pos([0.01, 0, 0], tracking=True) is True
    assert state["position"] == [0.01, 0, 0]
    module.drd.trackPos = lambda _: -1
    with pytest.raises(RuntimeError, match="trackPos failed"):
        robot.set_pos([0.01, 0, 0], tracking=True)
    with pytest.raises(ValueError, match="three finite"):
        robot.set_pos([float("inf"), 0, 0], tracking=True)


def test_tracking_limits_verify_sdk_status_and_readback(simulated_robot):
    robot, module, _state = simulated_robot
    robot.device_id = 7
    state = {"status": 0, "params": (0.01, 0.1, 1.0, 0)}

    def set_params(v, a, j, device):
        assert device == 7
        return state["status"]

    module.drd.setPosTrackParam = set_params
    module.drd.getPosTrackParam = lambda device: state["params"]
    assert robot.set_pos_param(.01, .1, 1.0, tracking=True) == (.01, .1, 1.0, tracking=True)
    state["status"] = -1
    with pytest.raises(RuntimeError, match="setPosTrackParam failed"):
        robot.set_pos_param(.01, .1, 1.0, tracking=True)
    state["status"] = 0
    state["params"] = (.02, .1, 1.0, 0)
    with pytest.raises(RuntimeError, match="exceeds approved"):
        robot.set_pos_param(.01, .1, 1.0, tracking=True)
    state["params"] = (.01, .1, 1.0, -1)
    with pytest.raises(RuntimeError, match="readback failed"):
        robot.set_pos_param(.01, .1, 1.0, tracking=True)
    with pytest.raises(ValueError, match="finite positive"):
        robot.set_pos_param(0, .1, 1.0, tracking=True)


def test_native_recording_reports_sdk_failure(simulated_robot):
    import threading
    import time as real_time

    robot, module, _state = simulated_robot
    module.time = real_time
    robot._is_open = True
    robot.device_id = 7
    robot._sdk_lock = threading.RLock()
    robot.logging_thread = None
    robot.logging_active = False
    robot.recording_error = None
    robot.log_data = []
    robot.log_freq = None
    module.dhd.getPosition = lambda buf: -1
    module.dhd.getLinearVelocity = lambda buf: 0
    module.dhd.getForce = lambda buf: 0

    robot.start_recording(freq=100)
    robot.logging_thread.join(timeout=1)
    assert not robot.logging_active
    with pytest.raises(RuntimeError, match="recording failed"):
        robot.check_recording_health()
    with pytest.raises(RuntimeError, match="recording failed"):
        robot.stop_recording()
    with pytest.raises(RuntimeError, match="incomplete recording"):
        robot.save_log("should_not_exist.h5")


def test_native_recording_success_uses_checked_vectors(simulated_robot):
    import threading
    import time as real_time

    robot, module, _state = simulated_robot
    module.time = real_time
    robot._is_open = True
    robot.device_id = 7
    robot._sdk_lock = threading.RLock()
    robot.logging_thread = None
    robot.logging_active = False
    robot.recording_error = None
    robot.log_data = []
    robot.log_freq = None

    def fill(buf, value):
        buf[:] = [value, value, value]
        return 0

    module.dhd.getPosition = lambda buf: fill(buf, 1.0)
    module.dhd.getLinearVelocity = lambda buf: fill(buf, 2.0)
    module.dhd.getForce = lambda buf: fill(buf, 3.0)
    robot.start_recording(freq=100)
    real_time.sleep(.04)
    robot.check_recording_health()
    robot.stop_recording()
    assert robot.log_data
    assert all(sample[1:] == [1.0] * 3 + [2.0] * 3 + [3.0] * 3
               for sample in robot.log_data)


def test_close_still_closes_after_recording_failure(simulated_robot):
    import threading

    robot, module, _state = simulated_robot
    robot._is_open = True
    robot.device_id = 7
    robot._sdk_lock = threading.RLock()
    robot.logging_thread = None
    robot.logging_active = False
    robot.recording_error = ValueError("simulated acquisition failure")
    robot.log_data = []
    calls = []
    module.drd.close = lambda device_id: calls.append(device_id) or 0
    # With no thread, close should still release the device.
    robot.close()
    assert calls == [7]
    assert not robot.is_open()


def test_recording_frequency_validation(simulated_robot):
    import threading
    robot, module, _state = simulated_robot
    robot._is_open = True
    robot._sdk_lock = threading.RLock()
    robot.logging_thread = None
    for freq in [0, -1, float("nan"), float("inf"), True]:
        with pytest.raises(ValueError, match="frequency"):
            robot.start_recording(freq=freq)


def test_original_getters_return_lists_and_reject_bad_reads(simulated_robot):
    robot, module, _state = simulated_robot
    robot.pos = [0.0, 0.0, 0.0]
    robot.vel = [0.0, 0.0, 0.0]
    robot.force = [0.0, 0.0, 0.0]

    def read(values, result):
        values[:] = result
        return 0

    module.dhd.getPosition = lambda values: read(values, [0.1, 0.2, 0.3])
    module.dhd.getLinearVelocity = lambda values: read(values, [0.0, 0.1, 0.0])
    module.dhd.getForce = lambda values: read(values, [0.5, 0.0, 0.0])

    # Fixture overrides the original get_pos() to simulate the preload spring.
    assert module.OmegaRobot.get_pos(robot) == [0.1, 0.2, 0.3]
    assert module.OmegaRobot.get_vel(robot) == [0.0, 0.1, 0.0]
    assert module.OmegaRobot.get_force(robot) == [0.5, 0.0, 0.0]

    module.dhd.getPosition = lambda values: -1
    with pytest.raises(RuntimeError, match="position read failed"):
        module.OmegaRobot.get_pos(robot)

    module.dhd.getLinearVelocity = lambda values: None
    with pytest.raises(RuntimeError, match="velocity read failed"):
        module.OmegaRobot.get_vel(robot)

    module.dhd.getForce = lambda values: read(values, [float("nan"), 0, 0])
    with pytest.raises(RuntimeError, match="non-finite"):
        module.OmegaRobot.get_force(robot)


def test_original_set_pos_checks_tracking_and_discrete_movement(simulated_robot):
    robot, module, state = simulated_robot
    calls = []

    def move(target, block=True):
        calls.append(("move", tuple(target), block))
        return 0

    def track(target):
        calls.append(("track", tuple(target)))
        return 0

    module.drd.moveToPos = move
    module.drd.trackPos = track
    assert robot.set_pos([0, 0.01, 0.02]) is True
    assert robot.set_pos([0.02, 0.03, 0], block=False) is True
    assert robot.set_pos([0, 0, 0.01], tracking=True) is True
    assert calls == [
        ("move", (0.0, 0.01, 0.02), True),
        ("move", (0.02, 0.03, 0.0), False),
        ("track", (0.0, 0.0, 0.01)),
    ]

    module.drd.moveToPos = lambda target, block=True: -1
    with pytest.raises(RuntimeError, match="moveToPos failed"):
        robot.set_pos([0, 0, 0])
    module.drd.trackPos = lambda target: None
    with pytest.raises(RuntimeError, match="trackPos failed"):
        robot.set_pos([0, 0, 0], tracking=True)
    with pytest.raises(RuntimeError, match="trackPos failed"):
        robot.set_pos([0, 0, 0], tracking=True)
    with pytest.raises(ValueError, match="three finite"):
        robot.set_pos([float("inf"), 0, 0], tracking=True)


def test_original_set_pos_params_validates_tracking_and_move(simulated_robot):
    robot, module, _state = simulated_robot
    robot.device_id = 7
    state = {"status": 0, "actual": (0.01, 0.1, 1.0, 0), "mode": None}

    def set_move(v, a, j, device_id):
        state["mode"] = "move"
        return state["status"]

    def set_track(v, a, j, device_id):
        state["mode"] = "track"
        return state["status"]

    module.drd.setPosMoveParam = set_move
    module.drd.setPosTrackParam = set_track
    module.drd.getPosMoveParam = lambda device_id: state["actual"]
    module.drd.getPosTrackParam = lambda device_id: state["actual"]

    assert robot.set_pos_param(0.01, 0.1, 1.0) is True
    assert state["mode"] == "move"
    assert robot.set_pos_param(0.01, 0.1, 1.0, tracking=True) is True
    assert state["mode"] == "track"
    assert robot.get_pos_param(tracking=True) == (0.01, 0.1, 1.0, 0)

    state["status"] = -1
    with pytest.raises(RuntimeError, match="setPosMoveParam failed"):
        robot.set_pos_param(0.01, 0.1, 1.0)
    state["status"] = 0
    state["actual"] = (0.02, 0.1, 1.0, 0)
    with pytest.raises(RuntimeError, match="exceeds approved"):
        robot.set_pos_param(0.01, 0.1, 1.0)
    state["actual"] = (0.01, 0.1, 1.0, None)
    with pytest.raises(RuntimeError, match="readback failed"):
        robot.set_pos_param(0.01, 0.1, 1.0, tracking=True)


def test_original_set_force_returns_acknowledgment_and_rejects_errors(simulated_robot):
    robot, module, _state = simulated_robot
    captured = []
    module.dhd.setForce = lambda target: captured.append(target) or 0
    assert robot.set_force([0, 0.1, 0]) == 0
    assert captured == [[0.0, 0.1, 0.0]]
    module.dhd.setForce = lambda target: -1
    with pytest.raises(RuntimeError, match="setForce failed"):
        robot.set_force([0, 0.1, 0])
