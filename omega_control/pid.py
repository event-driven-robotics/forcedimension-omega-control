"""Small, hardware-independent PID primitives used by omega-control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class PreloadStatus(str, Enum):
    """Terminal states for a preload acquisition."""

    SETTLED = "settled"
    TIMEOUT = "timeout"
    MAX_DISPLACEMENT = "max_displacement"
    MAX_FORCE = "max_force"
    INVALID_MEASUREMENT = "invalid_measurement"
    DEVICE_ERROR = "device_error"


@dataclass(frozen=True)
class PIDConfig:
    """Configuration for a PID whose output is bounded axis velocity."""

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_limit: float = 0.002
    integral_limit: float = 100.0
    derivative_filter_tau: float = 0.01

    def __post_init__(self) -> None:
        values = (
            self.kp,
            self.ki,
            self.kd,
            self.output_limit,
            self.integral_limit,
            self.derivative_filter_tau,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("PID parameters must be finite")
        if self.kp < 0.0 or self.ki < 0.0 or self.kd < 0.0:
            raise ValueError("PID gains must be non-negative")
        if self.output_limit <= 0.0:
            raise ValueError("output_limit must be positive")
        if self.integral_limit < 0.0:
            raise ValueError("integral_limit must be non-negative")
        if self.derivative_filter_tau < 0.0:
            raise ValueError("derivative_filter_tau must be non-negative")


@dataclass(frozen=True)
class PIDOutput:
    """One PID update, including terms useful for diagnostics."""

    output: float
    error: float
    proportional: float
    integral: float
    derivative: float
    measurement_rate: float
    saturated: bool


class PIDController:
    """PID controller with derivative-on-measurement and anti-windup."""

    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        """Clear the integral and derivative state."""

        self._integral_error = 0.0
        self._measurement_rate = 0.0
        self._previous_measurement: float | None = None

    def update(self, setpoint: float, measurement: float, dt: float) -> PIDOutput:
        """Advance the controller by ``dt`` seconds."""

        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        if not math.isfinite(setpoint) or not math.isfinite(measurement):
            raise ValueError("setpoint and measurement must be finite")

        error = setpoint - measurement
        proportional = self.config.kp * error

        if self._previous_measurement is None:
            raw_measurement_rate = 0.0
        else:
            raw_measurement_rate = (
                measurement - self._previous_measurement
            ) / dt
        self._previous_measurement = measurement

        tau = self.config.derivative_filter_tau
        alpha = 1.0 if tau == 0.0 else dt / (tau + dt)
        self._measurement_rate += alpha * (
            raw_measurement_rate - self._measurement_rate
        )
        derivative = -self.config.kd * self._measurement_rate

        proposed_integral_error = self._integral_error + error * dt
        proposed_integral_error = max(
            -self.config.integral_limit,
            min(self.config.integral_limit, proposed_integral_error),
        )
        proposed_integral = self.config.ki * proposed_integral_error
        proposed_output = proportional + proposed_integral + derivative
        output = max(
            -self.config.output_limit,
            min(self.config.output_limit, proposed_output),
        )
        saturated = output != proposed_output

        # Do not accumulate integral error when it would drive the output
        # farther into saturation. Integration resumes as soon as it helps unwind.
        drives_further_into_saturation = saturated and error * proposed_output > 0.0
        if not drives_further_into_saturation:
            self._integral_error = proposed_integral_error

        integral = self.config.ki * self._integral_error
        unsaturated_output = proportional + integral + derivative
        output = max(
            -self.config.output_limit,
            min(self.config.output_limit, unsaturated_output),
        )

        return PIDOutput(
            output=output,
            error=error,
            proportional=proportional,
            integral=integral,
            derivative=derivative,
            measurement_rate=self._measurement_rate,
            saturated=output != unsaturated_output,
        )
