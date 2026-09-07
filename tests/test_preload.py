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
