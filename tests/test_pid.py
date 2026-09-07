"""Unit tests for the hardware-independent PID controller."""

import sys
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).parents[1] / "omega_control" / "pid.py"
SPEC = spec_from_file_location("omega_control_pid_for_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
pid = module_from_spec(SPEC)
sys.modules[SPEC.name] = pid
SPEC.loader.exec_module(pid)


def test_proportional_output_and_limit():
    controller = pid.PIDController(pid.PIDConfig(kp=2.0, output_limit=1.0))

    update = controller.update(setpoint=1.0, measurement=0.0, dt=0.1)

    assert update.proportional == pytest.approx(2.0)
    assert update.output == pytest.approx(1.0)
    assert update.saturated


def test_integral_accumulates_without_saturation():
    controller = pid.PIDController(
        pid.PIDConfig(kp=0.0, ki=2.0, output_limit=10.0)
    )

    first = controller.update(setpoint=1.0, measurement=0.0, dt=0.25)
    second = controller.update(setpoint=1.0, measurement=0.0, dt=0.25)

    assert first.integral == pytest.approx(0.5)
    assert second.integral == pytest.approx(1.0)


def test_integral_does_not_wind_up_while_saturated():
    controller = pid.PIDController(
        pid.PIDConfig(kp=2.0, ki=1.0, output_limit=1.0)
    )

    for _ in range(20):
        update = controller.update(setpoint=1.0, measurement=0.0, dt=0.1)

    assert update.integral == pytest.approx(0.0)
    assert update.output == pytest.approx(1.0)


def test_derivative_is_taken_on_measurement():
    controller = pid.PIDController(
        pid.PIDConfig(
            kp=0.0,
            kd=0.5,
            output_limit=10.0,
            derivative_filter_tau=0.0,
        )
    )
    controller.update(setpoint=0.0, measurement=0.0, dt=0.1)

    update = controller.update(setpoint=0.0, measurement=1.0, dt=0.1)

    assert update.measurement_rate == pytest.approx(10.0)
    assert update.derivative == pytest.approx(-5.0)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"kp": -1.0},
        {"kp": 1.0, "output_limit": 0.0},
        {"kp": 1.0, "derivative_filter_tau": -0.1},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        pid.PIDConfig(**kwargs)
