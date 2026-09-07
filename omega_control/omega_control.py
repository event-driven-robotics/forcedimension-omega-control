"""Force Dimension Omega Robot Control Interface.

This module provides a unified interface for controlling and reading the state
of a Force Dimension Omega haptic robot via the DHD/DRD APIs. It includes
threaded data recording capabilities for position, velocity, and force data.

The main class OmegaRobot handles device initialization, connection management,
motion control, trajectory generation, and real-time data logging to HDF5 files.

Example:
    Basic usage of the robot control interface::

        from omega_control import OmegaRobot
        
        robot = OmegaRobot(verbose=True)
        robot.to_origin()
        robot.start_recording(freq=1000)
        # ... perform robot operations ...
        robot.stop_recording()
        
        # Save to file
        robot.save_log("session_data.h5")
        
        # Or get data directly
        data, columns = robot.return_log()
        
        robot.close()

Authors:
    Simon Mueller-Cleve (simon.mullercleve@iit.it)
    Quentin Halbach (halbach.quentin@iit.it)

Institution:
    Istituto Italiano di Tecnologia (IIT)

Created:
    2025-10-30

Last Modified:
    2025-11-17
"""

import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime

from omega_control._sdk_config import configure_forcedimension_sdk

configure_forcedimension_sdk()

import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd
import h5py
import matplotlib.pyplot as plt
import numpy as np

from omega_control.pid import PIDConfig, PIDController, PreloadStatus


@dataclass(frozen=True)
class PreloadSample:
    """One sample captured by the preload force-control loop."""

    time: float
    setpoint: float
    measured_force: float
    filtered_force: float
    error: float
    proportional: float
    integral: float
    derivative: float
    force_rate: float
    velocity: float
    commanded_position: float
    saturated: bool


@dataclass(frozen=True)
class PreloadResult:
    """Outcome and telemetry from a preload acquisition."""

    status: PreloadStatus
    target_axis: str
    target_force: float
    final_position: float
    final_force: float
    elapsed_time: float
    iterations: int
    samples: tuple[PreloadSample, ...]
    message: str = ""

    @property
    def reached(self) -> bool:
        """Whether the requested preload settled successfully."""

        return self.status is PreloadStatus.SETTLED


class OmegaRobot:
    """
    Interface class for controlling and reading state from a Force Dimension robot.

    Handles device initialization, connection management, motion control,
    and data recording for position, velocity, and force.

    Attributes
    ----------
    device_id : int or None
        The unique identifier of the opened device.
    _is_open : bool
        Indicates whether the device connection is currently open.
    pos : list[float]
        The current Cartesian position [x, y, z].
    vel : list[float]
        The current Cartesian velocity [vx, vy, vz].
    force : list[float]
        The current Cartesian force [fx, fy, fz].
    verbose : bool
        Whether to print detailed information about operations.
    outpath : str
        Default output path for saving recorded data (HDF5 format).
    logging_thread : threading.Thread or None
        Background thread for data recording.
    logging_active : bool
        Flag indicating if data recording is currently active.
    log_data : list
        Recorded data samples [time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, force_x, force_y, force_z].
    log_freq : float or None
        Recording frequency in Hz, or None for maximum speed.
    default_move_param : tuple
        Default motion trajectory parameters (vmax, amax, jerk).
    default_track_param : tuple
        Default tracking trajectory parameters (vmax, amax, jerk).
    """

    def __init__(self, verbose=False, outpath="./"):
        """
        Initialize the OmegaRobot object.

        Opens a connection to the device, initializes it, and
        prepares state-tracking vectors for position, velocity, and force.

        Parameters
        ----------
        verbose : bool, optional
            If True, prints information about device operations.
        outpath : str, optional
            Default file path to save recorded data in HDF5 format.
        """
        self.verbose = verbose
        self.outpath = outpath
        self.device_id = None
        self._is_open = False
        self.open()
        self.initialize()

        self.default_move_param = self.get_pos_param()
        self.default_track_param = self.get_pos_param(tracking=True)

        # Initialize state variables
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.force = [0.0, 0.0, 0.0]

        # Data logging attributes
        self.logging_thread = None
        self.logging_active = False
        self.log_data = []
        self.log_freq = None  # Hz (None = as fast as possible)

    # -------------------------------------------------------------------------
    # DEVICE CONTROL METHODS
    # -------------------------------------------------------------------------

    def open(self):
        """
        Open a connection to the first available device connected to
        the system. The order in which devices are opened persists
        until devices are added or removed.

        If this call is successful, the default device ID is set to
        the newly opened device. See the :ref:`multiple_devices`
        section for more information on using multiple devices on
        the same computer.

        :returns: The device ID on success, -1 otherwise.
        """

        if self._is_open:
            if self.verbose:
                print("Device already open.")
            return

        device_id = drd.open()
        if device_id < 0:
            raise RuntimeError("Failed to open device.")

        self.device_id = device_id
        self._is_open = True
        if self.verbose:
            print("Device opened successfully.")

    def is_open(self):
        """
        Checks whether a connection to the device is currently open.

        This method returns the current connection state of the device. 
        A device is considered open after a successful call to :meth:`open`, 
        and remains open until it is explicitly closed or an error occurs.

        :returns:
            True if the device connection is open, False otherwise.
        """

        return self._is_open

    def force_init(self, precision: bool = False):
        """
        Forces re-initialization of the Force Dimension device.

        This function re-initializes the device regardless of its current state.
        If the ``precision`` flag is set, the function performs a precision 
        initialization by moving each axis to a known reference position and 
        resetting encoder counters. Otherwise, a standard automatic 
        initialization is performed.

        :param bool precision:
            If True, performs precision initialization; otherwise performs
            standard automatic initialization.

        :raises RuntimeError:
            If re-initialization fails.

        :returns:
            True on success.
        """

        try:
            if precision:
                drd.precisionInit(self.device_id)
            else:
                drd.autoInit(self.device_id)

            if self.verbose:
                print("Force Dimension device re-initialized successfully.")
            return True

        except Exception as e:
            drd.close(self.device_id)
            raise RuntimeError(
                f"Failed to force re-initialize the device: {e}") from e

    def initialize(self, precision: bool = False):
        """
        Automatically initializes the robot by moving it to a known position 
        and resetting encoder counters to their correct values.

        If the ``precision`` flag is set, this function performs a precision 
        initialization for supported devices by moving each axis to a known 
        position, resetting its encoder counters, and validating each axis 
        position against a reference.

        :param bool precision: 
            If True, performs precision initialization; otherwise performs 
            standard initialization.

        :raises RuntimeError: 
            If the robot fails to initialize.

        :returns: 
            True on success.
        """

        try:
            if not drd.isInitialized() and not precision:
                drd.autoInit(self.device_id)
            elif precision:
                drd.precisionInit(self.device_id)
            else:
                drd.start()

            if self.verbose:
                print("Robot successfully initialized.")
            return True

        except Exception as e:
            drd.close(self.device_id)
            raise RuntimeError(f"Failed to initialize the robot: {e}") from e

    def check_init(self):
        """
        Verify that the robot has been properly initialized.

        This function sweeps all endstops and compares the measured joint-space
        positions to the expected values stored in the device's memory. If the
        robot is not yet initialized, it will first perform automatic initialization
        (equivalent to :func:`initialize(precision=False)`) before running
        the endstop check.

        Returns
        -------
        int
            0 on success, -1 otherwise.
        """

        result = drd.checkInit(self.device_id)
        return result

    def restart(self) -> bool:
        """
        Restarts the robotic control loop for the current device.

        This function restarts the control loop for the robot associated 
        with ``self.device_id``. The device must be initialized before this 
        function can be called successfully.

        :returns:
            True if the restart was successful, False otherwise.

        :raises RuntimeError:
            If the device fails to restart.
        """

        try:
            result = drd.start(self.device_id)
            if result == 0:
                if self.verbose:
                    print("Device restarted successfully.")
                return True
            else:
                if self.verbose:
                    print("Device restart failed.")
                return False

        except Exception as e:
            if self.verbose:
                print(f"Error restarting device: {e}")
            raise RuntimeError(f"Failed to restart the device: {e}") from e

    def get_device_id(self):
        """
        Get the ID of the current default device.

        :returns:
            The ID of the current default device
        """

        return self.device_id

    def get_serial_number(self):
        """
        Return the device serial number.

        :returns:
            The serial number on success.
        """

        serial_number = dhd.getSerialNumber(self.device_id)
        return serial_number

    def get_control_freq(self):
        """
        This function returns the average refresh rate of the control loop
        (in [kHz]) since the function was last called.

        :returns:
            The control frequency on success, and -1.0 otherwise.
        """

        ctrl_frequ = drd.getCtrlFreq(self.device_id)
        return ctrl_frequ

    def stop(self):
        """
        Stop the device. This routine disables the force on the
        haptic device and puts it into BRAKE mode.
        """

        dhd.stop(self.device_id)

    def is_moving(self):
        """
        Checks whether the particular robot is moving (following a
        call to
        :func:`move_to_pos()`,
        :func:`track_pos()` or
        as opposed to holding the target position after successfully
        reaching it.

        :returns:
            ``True`` if the robot is moving, ``False`` otherwise.
        """

        is_moving = drd.isMoving(self.device_id)
        return is_moving

    def set_gravity_compensation(self, bool=True):
        """
        Enable/disable gravity compensation.

        :param bool enable:
            ``True`` to turn on gravity compensation, ``False`` to
            turn off gravity compensation.

        :returns:
            0 on success, -1 otherwise.
        """

        result = dhd.setGravityCompensation(self.device_id, bool)
        if self.verbose:
            print(f"Gravity compensation set to {bool}")
        return result

    def set_effector_mass(self, mass):
        """
        Define the mass of the end-effector (in [kg]). This function
        is required to  provide accurate gravity compensation when
        custom-made or modified end-effectors are used.

        :param float mass:
            The actual end-effector mass (in [kg]).

        :returns:
            0 on success, -1 otherwise.
        """

        result = dhd.setEffectorMass(self.device_id, mass)
        if self.verbose:
            print(f"New end-effector mass set to {mass}")
        return result

    def get_max_force(self):
        """
        Retrieve the current limit (in [N]) to the force magnitude
        that can be applied by the haptic device. If the return value
        if negative, the limit is disabled and the full range of force
        available can be applied.

        :returns:
            The current force limit (in N) if set, and ``-1.0`` if no
            limit is enforced.
        """

        max_force = dhd.getMaxForce(self.device_id)
        return max_force

    def set_max_force(self, limit):
        """
        Define a limit (in [N]) to the magnitude of the force that
        can be applied to the haptic device. The limit applies to all
        :func:`forcedimension_core.dhd.setForce()` and related calls,
        and ensures the force applied to the device end-effector
        remains below the requested value. If a negative limit is
        set,  there is no max force and the full range of force can
        be applied.

        Note
        ----
        The force limit enforced only applies to forces set
        programatically by dhd.libdhd.setForce(). Setting DAC values
        directly will bypass this limit.


        :param float limit:
            Max magnitude of force that can be applied (in [N]).

        :returns:
            0 on success, -1 otherwise.
        """

        result = dhd.setMaxForce(self.device_id, limit)
        if self.verbose:
            print(f"Maximum output force set to {limit}")
        return result

    def to_origin(self):
        """
        Moves the robot end-effector to the origin position [0.0, 0.0, 0.0].

        This function commands the robot to move to its reference (home) 
        position at the Cartesian origin. It uses :meth:`set_pos` internally 
        to perform the movement.

        :raises RuntimeError:
            If the movement command fails.
        """

        self.set_pos([0.0, 0.0, 0.0])

    def get_pos(self):
        """
        Retrieve the position of the end-effector about the X, Y, and
        Z axes. Please refer to your device user manual for more
        information on your device coordinate system.
        """

        dhd.getPosition(self.pos)
        return self.pos

    def get_vel(self):
        """
        Retrieve the estimated instanteous linear velocity (in [m/s]).
        """

        dhd.getLinearVelocity(self.vel)
        return self.vel

    def get_force(self):
        """
        Retrieve the force vector applied to the end-effector (in
        [N]) about the X, Y, and Z axes Please refer to your device
        user manual for more information on your device coordinate
        system.
        """

        dhd.getForce(self.force)
        return self.force

    def set_pos(self, target: list[float], block: bool = True, tracking: bool = False):
        """
        Moves the robot end-effector to a desired Cartesian position.

        The motion follows a straight-line trajectory with smooth 
        acceleration and deceleration profiles. These profiles can be tuned 
        through the device's trajectory generation parameters.

        .. note::
            The motion paths generated by this function are **not guaranteed 
            to be continuous** if a command is interrupted by another call to 
            this function while still in progress.  
            To ensure trajectory continuity, use 
            :func:`forcedimension_core.drd.trackPos()`.  
            See the :ref:`regulation` section for more details.

        If ``tracking`` is True, the function commands a continuous position 
        update instead of a blocking movement.

        :param list[float] target:
            Target Cartesian position ``[px, py, pz]`` in meters, where each 
            component corresponds to the X, Y, and Z axes respectively.

        :param bool block:
            If True (default), the function blocks until motion is complete.  
            If False, the command returns immediately.

        :param bool tracking:
            If True, enables continuous trajectory tracking mode instead of 
            a discrete move.

        :raises RuntimeError:
            If the command fails or the robot is not properly initialized.

        :returns:
            True on success.
            
        Notes
        -----
        - Use `block=True` for discrete point-to-point movements
        - Use `tracking=True` for smooth continuous trajectories
        - Position coordinates are in meters relative to device origin
        - Z-axis typically points upward from the device base
        
        Examples
        --------
        >>> # Move to a specific position and wait for completion
        >>> robot.set_pos([0.05, 0.02, -0.01])  # 5cm right, 2cm forward, 1cm down
        >>> 
        >>> # Start movement but don't wait for completion
        >>> robot.set_pos([0.0, 0.0, 0.05], block=False)
        >>> 
        >>> # Continuous tracking (for smooth trajectories)
        >>> robot.set_pos([0.03, 0.0, 0.0], tracking=True)
        """
        try:
            if not tracking:
                drd.moveToPos(target, block=block)
                if self.verbose:
                    print(f"Moving to {target} (block={block})")
            else:
                drd.trackPos(target)
                if self.verbose:
                    print(f"Tracking position {target}")

            return True

        except Exception as e:
            if self.verbose:
                print(f"Failed to move to {target}: {e}")
            raise RuntimeError(
                f"Failed to move to target position: {e}") from e

    def hold(self):
        """
        Immediately make the robot hold its current position. All
        motion commands are abandoned.

        :returns:
            0 on success, and -1 otherwise.
        """

        result = drd.hold(self.device_id)
        return result

    def get_pos_param(self, tracking: bool = False):
        """
        Retrieves the Cartesian trajectory generation parameters for the robot.

        Depending on the ``tracking`` flag, this method returns either the 
        standard motion trajectory parameters or the tracking trajectory 
        parameters currently configured on the device.

        :param bool tracking:
            If False (default), retrieves standard motion parameters using 
            :func:`forcedimension_core.drd.getPosMoveParam`.  
            If True, retrieves tracking trajectory parameters using 
            :func:`forcedimension_core.drd.getPosTrackParam`.

        :returns:
            A tuple ``(v_max, a_max, jerk_max, err)``, where:

            - **v_max** (*float*): Maximum velocity in m/s  
            - **a_max** (*float*): Maximum acceleration in m/s²  
            - **jerk_max** (*float*): Maximum jerk in m/s³  
            - **err** (*int*): 0 on success, -1 otherwise

        :raises RuntimeError:
            If parameter retrieval fails.
        """

        try:
            if not tracking:
                params = drd.getPosMoveParam(self.device_id)
                if self.verbose:
                    print(
                        f"Retrieved motion parameters: "
                        f"v_max={params[0]:.3f} m/s, a_max={params[1]:.3f} m/s², jerk={params[2]:.3f} m/s³."
                    )
            else:
                params = drd.getPosTrackParam(self.device_id)
                if self.verbose:
                    print(
                        f"Retrieved tracking parameters: "
                        f"v_max={params[0]:.3f} m/s, a_max={params[1]:.3f} m/s², jerk={params[2]:.3f} m/s³."
                    )

            return params

        except Exception as e:
            if self.verbose:
                print(f"Failed to retrieve position parameters: {e}")
            raise RuntimeError(
                f"Failed to retrieve position parameters: {e}") from e

    def set_pos_param(self, vmax: float, amax: float, jerk: float, tracking: bool = False) -> bool:
        """
        Sets the Cartesian trajectory generation parameters for the robot.

        This function updates the motion or tracking trajectory generation
        parameters that control the maximum velocity, acceleration, and jerk 
        of the end-effector motion.

        :param float vmax:
            Maximum velocity in meters per second (m/s).

        :param float amax:
            Maximum acceleration in meters per second squared (m/s²).

        :param float jerk:
            Maximum jerk in meters per second cubed (m/s³).

        :param bool tracking:
            If False (default), sets standard motion parameters using 
            :func:`forcedimension_core.drd.setPosMoveParam`.  
            If True, sets tracking parameters using 
            :func:`forcedimension_core.drd.setPosTrackParam`.

        :returns:
            True on success, False otherwise.

        :raises RuntimeError:
            If parameter setting fails.
            
        Notes
        -----
        - Lower values create smoother, slower movements
        - Higher values create faster but potentially less smooth movements
        - Jerk limits control smoothness of acceleration changes
        - Use different parameters for motion vs tracking modes
        
        Examples
        --------
        >>> # Slow, smooth movements for precision tasks
        >>> robot.set_pos_param(vmax=0.1, amax=0.5, jerk=2.0)
        >>> 
        >>> # Fast movements for efficiency
        >>> robot.set_pos_param(vmax=0.5, amax=2.0, jerk=10.0)
        >>> 
        >>> # Set tracking parameters for continuous motion
        >>> robot.set_pos_param(vmax=0.2, amax=1.0, jerk=5.0, tracking=True)
        """

        try:
            if not tracking:
                drd.setPosMoveParam(vmax, amax, jerk, self.device_id)
                if self.verbose:
                    print(
                        f"Set motion parameters: "
                        f"v_max={vmax:.3f} m/s, a_max={amax:.3f} m/s², jerk={jerk:.3f} m/s³."
                    )
            else:
                drd.setPosTrackParam(vmax, amax, jerk, self.device_id)
                if self.verbose:
                    print(
                        f"Set tracking parameters: "
                        f"v_max={vmax:.3f} m/s, a_max={amax:.3f} m/s², jerk={jerk:.3f} m/s³."
                    )

            return True

        except Exception as e:
            if self.verbose:
                print(f"Failed to set position parameters: {e}")
            raise RuntimeError(
                f"Failed to set position parameters: {e}") from e

    def set_default_pos_param(self, tracking: bool = False) -> bool:
        """
        Resets the Cartesian trajectory generation parameters to default values.

        This function restores either the standard motion parameters or the 
        tracking parameters to their predefined defaults.

        :param bool tracking:
            If False (default), resets standard motion parameters.  
            If True, resets tracking parameters.

        :returns:
            True on success, False otherwise.

        :raises RuntimeError:
            If setting default parameters fails.
        
        :Note:
            default parameters are:
            - vmax = 1.0 m/s
            - amax = 1.0 m/s2
            - jerk = 1.0 m/s3
        """
        try:
            if not tracking:
                self.set_pos_param(
                    vmax=self.default_move_param[0],
                    amax=self.default_move_param[1],
                    jerk=self.default_move_param[2],
                    tracking=False
                )
                if self.verbose:
                    print("Set position parameters to default for motion.")
            else:
                self.set_pos_param(
                    vmax=self.default_track_param[0],
                    amax=self.default_track_param[1],
                    jerk=self.default_track_param[2],
                    tracking=True
                )
                if self.verbose:
                    print("Set position parameters to default for tracking.")

            return True

        except Exception as e:
            if self.verbose:
                print(f"Failed to set default position parameters: {e}")
            raise RuntimeError(
                f"Failed to set default position parameters: {e}") from e

    def set_force(self, target):
        """
        Set the desired force (in [N]) about the X, Y, and Z axes to
        be applied to the end-effector of the device.

        :param VectorLike f:
            Translation force vector (fx, fy, fz) (in [N]).
        """

        dhd.setForce(target)
        if self.verbose:
            print(f"Setting target force to {target}.")

    def preload(
        self,
        target_axis: str,
        target_force: float,
        *,
        kp: float = 1e-3,
        ki: float = 0.0,
        kd: float = 0.0,
        control_freq: float = 250.0,
        tolerance: float = 0.05,
        settle_time: float = 0.25,
        timeout: float = 10.0,
        max_velocity: float = 0.002,
        max_displacement: float = 0.01,
        max_force: float | None = None,
        force_filter_tau: float = 0.01,
        derivative_filter_tau: float = 0.01,
        force_ramp_rate: float = 2.0,
        integral_limit: float = 10.0,
        control_direction: int = 1,
        settle_force_rate: float | None = None,
        plot_results: bool = False,
    ) -> PreloadResult:
        """Acquire a force preload using a PID outer position-control loop.

        The PID output is an axis velocity in m/s. It is integrated into a
        non-blocking DRD position-tracking target, leaving the device's position
        regulator as the inner loop. The method returns after the target force
        has remained within ``tolerance`` for ``settle_time``.

        ``control_direction`` describes the local force/position relationship:
        use ``1`` when increasing position increases measured force, and ``-1``
        when it decreases measured force. Determine this conservatively for the
        end-effector and contact geometry before using a substantial preload.

        The returned :class:`PreloadResult` distinguishes successful settling,
        timeout, force and travel limits, invalid measurements, and device errors.
        On every exit the current position is held.
        """

        if (
            not isinstance(target_axis, str)
            or len(target_axis) != 1
            or target_axis.lower() not in "xyz"
        ):
            raise ValueError("target_axis must be one of 'x', 'y', or 'z'")
        target_axis = target_axis.lower()
        axis_idx = {"x": 0, "y": 1, "z": 2}[target_axis]

        numeric_parameters = {
            "target_force": target_force,
            "control_freq": control_freq,
            "tolerance": tolerance,
            "settle_time": settle_time,
            "timeout": timeout,
            "max_velocity": max_velocity,
            "max_displacement": max_displacement,
            "force_filter_tau": force_filter_tau,
            "force_ramp_rate": force_ramp_rate,
            "integral_limit": integral_limit,
        }
        if not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in numeric_parameters.values()
        ):
            raise ValueError("preload parameters must be finite numbers")
        if control_freq <= 0.0:
            raise ValueError("control_freq must be positive")
        if tolerance < 0.0 or settle_time < 0.0:
            raise ValueError("tolerance and settle_time must be non-negative")
        if timeout <= 0.0:
            raise ValueError("timeout must be positive")
        if max_velocity <= 0.0 or max_displacement <= 0.0:
            raise ValueError("max_velocity and max_displacement must be positive")
        if force_filter_tau < 0.0 or force_ramp_rate <= 0.0:
            raise ValueError("filter time must be non-negative and ramp rate positive")
        if control_direction not in (-1, 1):
            raise ValueError("control_direction must be 1 or -1")
        if settle_force_rate is not None and (
            not math.isfinite(settle_force_rate) or settle_force_rate < 0.0
        ):
            raise ValueError("settle_force_rate must be non-negative or None")

        if max_force is None:
            max_force = max(2.0 * abs(target_force), abs(target_force) + 1.0)
        if not math.isfinite(max_force) or max_force <= 0.0:
            raise ValueError("max_force must be finite and positive")

        controller = PIDController(
            PIDConfig(
                kp=kp,
                ki=ki,
                kd=kd,
                output_limit=max_velocity,
                integral_limit=integral_limit,
                derivative_filter_tau=derivative_filter_tau,
            )
        )
        period = 1.0 / control_freq
        started = time.monotonic()
        previous_time = started
        next_deadline = started
        samples: list[PreloadSample] = []
        status = PreloadStatus.DEVICE_ERROR
        message = ""
        final_position = math.nan
        final_force = math.nan
        initial_position = math.nan
        commanded_position = [math.nan, math.nan, math.nan]
        filtered_force = math.nan
        ramped_setpoint = math.nan
        settled_for = 0.0

        try:
            commanded_position = self.get_pos().copy()
            initial_position = commanded_position[axis_idx]
            initial_force = self.get_force()[axis_idx]
            final_force = initial_force
            if not math.isfinite(initial_position) or not math.isfinite(initial_force):
                status = PreloadStatus.INVALID_MEASUREMENT
                message = "Initial position or force measurement is not finite"
            else:
                filtered_force = initial_force
                ramped_setpoint = initial_force

                while True:
                    now = time.monotonic()
                    elapsed = now - started
                    if elapsed >= timeout:
                        status = PreloadStatus.TIMEOUT
                        message = f"Preload did not settle within {timeout:.3f} s"
                        break

                    actual_dt = now - previous_time
                    dt = period if not samples else min(actual_dt, 2.0 * period)
                    dt = max(dt, period * 0.1)
                    previous_time = now

                    measured_force = self.get_force()[axis_idx]
                    final_force = measured_force
                    if not math.isfinite(measured_force):
                        status = PreloadStatus.INVALID_MEASUREMENT
                        message = "Force measurement is not finite"
                        break
                    if abs(measured_force) > max_force:
                        status = PreloadStatus.MAX_FORCE
                        message = (
                            f"Measured force {measured_force:.4f} N exceeded "
                            f"the {max_force:.4f} N safety limit"
                        )
                        break

                    ramp_delta = max(
                        -force_ramp_rate * dt,
                        min(force_ramp_rate * dt, target_force - ramped_setpoint),
                    )
                    ramped_setpoint += ramp_delta

                    filter_alpha = (
                        1.0
                        if force_filter_tau == 0.0
                        else dt / (force_filter_tau + dt)
                    )
                    filtered_force += filter_alpha * (
                        measured_force - filtered_force
                    )
                    update = controller.update(ramped_setpoint, filtered_force, dt)
                    velocity = control_direction * update.output
                    proposed_position = commanded_position[axis_idx] + velocity * dt

                    if abs(proposed_position - initial_position) > max_displacement:
                        status = PreloadStatus.MAX_DISPLACEMENT
                        message = (
                            f"Preload would exceed the {max_displacement:.6f} m "
                            "travel limit"
                        )
                        break

                    commanded_position[axis_idx] = proposed_position
                    tracking_result = drd.trackPos(commanded_position)
                    if isinstance(tracking_result, (int, float)) and tracking_result < 0:
                        raise RuntimeError("DRD rejected the position tracking command")

                    final_position = commanded_position[axis_idx]
                    samples.append(
                        PreloadSample(
                            time=elapsed,
                            setpoint=ramped_setpoint,
                            measured_force=measured_force,
                            filtered_force=filtered_force,
                            error=update.error,
                            proportional=update.proportional,
                            integral=update.integral,
                            derivative=update.derivative,
                            force_rate=update.measurement_rate,
                            velocity=velocity,
                            commanded_position=final_position,
                            saturated=update.saturated,
                        )
                    )

                    at_final_setpoint = math.isclose(
                        ramped_setpoint,
                        target_force,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                    force_is_steady = (
                        settle_force_rate is None
                        or abs(update.measurement_rate) <= settle_force_rate
                    )
                    if (
                        at_final_setpoint
                        and abs(target_force - filtered_force) <= tolerance
                        and force_is_steady
                    ):
                        settled_for += dt
                    else:
                        settled_for = 0.0

                    if settled_for >= settle_time:
                        status = PreloadStatus.SETTLED
                        message = (
                            f"Target force remained within {tolerance:.4f} N "
                            f"for {settle_time:.3f} s"
                        )
                        break

                    next_deadline += period
                    sleep_for = next_deadline - time.monotonic()
                    if sleep_for > 0.0:
                        time.sleep(sleep_for)
                    else:
                        # Resynchronise after an overrun instead of accumulating lag.
                        next_deadline = time.monotonic()

        except Exception as exc:
            status = PreloadStatus.DEVICE_ERROR
            message = f"Preload control failed: {exc}"
        finally:
            try:
                hold_result = self.hold()
                if isinstance(hold_result, (int, float)) and hold_result < 0:
                    raise RuntimeError("DRD failed to hold the current position")
            except Exception as exc:
                if message:
                    message += f"; also failed to hold position: {exc}"
                else:
                    message = f"Failed to hold position: {exc}"
                status = PreloadStatus.DEVICE_ERROR

            # Prefer actual post-hold state in the result, but retain the last
            # valid control-loop values if the device cannot be read again.
            try:
                observed_position = self.get_pos()[axis_idx]
                observed_force = self.get_force()[axis_idx]
                if math.isfinite(observed_position):
                    final_position = observed_position
                if math.isfinite(observed_force):
                    final_force = observed_force
            except Exception:
                pass

        elapsed_time = time.monotonic() - started
        if math.isnan(final_position) and math.isfinite(initial_position):
            final_position = initial_position
        result = PreloadResult(
            status=status,
            target_axis=target_axis,
            target_force=target_force,
            final_position=final_position,
            final_force=final_force,
            elapsed_time=elapsed_time,
            iterations=len(samples),
            samples=tuple(samples),
            message=message,
        )

        if self.verbose:
            print(
                f"Preload {result.status.value}: force={result.final_force:.4f} N, "
                f"position={result.final_position:.6f} m, "
                f"elapsed={result.elapsed_time:.3f} s"
            )
        if plot_results:
            self._plot_preload_results(result)
        return result

    def _plot_preload_results(
        self,
        result: PreloadResult,
        save: bool = False,
        filename: str | None = None,
    ) -> None:
        """Plot force, PID output, and commanded position for a preload."""

        times = [sample.time for sample in result.samples]
        measured = [sample.measured_force for sample in result.samples]
        filtered = [sample.filtered_force for sample in result.samples]
        setpoints = [sample.setpoint for sample in result.samples]
        velocities = [sample.velocity for sample in result.samples]
        positions = [sample.commanded_position for sample in result.samples]

        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        axes[0].plot(times, measured, alpha=0.45, label="measured")
        axes[0].plot(times, filtered, label="filtered")
        axes[0].plot(times, setpoints, linestyle="--", label="setpoint")
        axes[0].set_ylabel("Force (N)")
        axes[0].legend()
        axes[0].grid(True, linestyle="--", alpha=0.4)

        axes[1].plot(times, velocities, color="tab:red")
        axes[1].axhline(0.0, color="gray", linewidth=1)
        axes[1].set_ylabel("Velocity (m/s)")
        axes[1].grid(True, linestyle="--", alpha=0.4)

        axes[2].plot(times, positions, color="tab:green")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel(f"{result.target_axis.upper()} target (m)")
        axes[2].grid(True, linestyle="--", alpha=0.4)

        fig.suptitle(
            f"PID preload: {result.status.value} "
            f"(target {result.target_force:.3f} N)"
        )
        fig.tight_layout()

        if save:
            if filename is None:
                filename = "preload_pid_convergence.png"
            fig.savefig(filename, dpi=300)
            plt.close(fig)
        else:
            plt.show()

    # -------------------------------------------------------------------------
    # DATA RECORDING METHODS
    # -------------------------------------------------------------------------

    def _record_loop(self):
        """
        Private thread function to record robot data continuously.

        Continuously logs timestamped position, velocity, and force data
        while `self.logging_active` is True. The logging frequency is 
        controlled by `self.log_freq`. If `log_freq` is None or 0, data 
        is recorded as fast as possible.

        Logged data format:
            [elapsed_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, force_x, force_y, force_z]
        """

        t0 = time.time()
        last_time = t0
        interval = 1.0 / self.log_freq if self.log_freq else 0.0

        if self.verbose:
            print(f"Recording started (freq={self.log_freq or 'max'} Hz)")

        while self.logging_active:
            now = time.time()
            if self.log_freq is None or (now - last_time) >= interval:
                self.get_pos()
                self.get_vel()
                self.get_force()
                self.log_data.append(
                    [now - t0] + self.pos + self.vel + self.force)
                last_time = now

            if self.log_freq:
                # Sleep a small amount to maintain the desired frequency
                elapsed = time.time() - last_time
                sleep_time = max(0.0, interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        if self.verbose:
            print(
                f"Recording stopped. Collected {len(self.log_data)} samples.")

    def start_recording(self, freq: float = None):
        """
        Start recording position, velocity, and force data in a background thread.

        Parameters
        ----------
        freq : float, optional
            Sampling frequency in Hz. If None, records data as fast as possible.

        Notes
        -----
        - Recorded data is stored in `self.log_data` as lists:
        [elapsed_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, force_x, force_y, force_z].
        - The recording runs in a daemon thread and stops when `self.logging_active` is set to False.
        - Higher frequencies (>1000 Hz) may impact system performance
        - Use moderate frequencies (100-1000 Hz) for most applications
        
        Examples
        --------
        >>> # Record at 1000 Hz for precise motion tracking
        >>> robot.start_recording(freq=1000)
        >>> 
        >>> # Record as fast as possible (typically >10 kHz)
        >>> robot.start_recording()
        >>> 
        >>> # Low frequency for long-duration experiments
        >>> robot.start_recording(freq=100)
        """

        if self.logging_active:
            if self.verbose:
                print("Recording already active.")
            return

        self.log_freq = freq
        self.log_data = []
        self.logging_active = True
        self.logging_thread = threading.Thread(
            target=self._record_loop, daemon=True)
        self.logging_thread.start()

        if self.verbose:
            print(f"Recording started (freq={freq or 'max'} Hz)")

    def stop_recording(self):
        """
        Stop the background data recording thread safely.

        Notes
        -----
        - Sets `self.logging_active` to False to terminate the recording loop.
        - Joins the background thread to ensure it finishes cleanly.
        - Resets `self.logging_thread` to None after stopping.
        """

        if not self.logging_active:
            if self.verbose:
                print("No active recording to stop.")
            return

        self.logging_active = False
        self.logging_thread.join()
        self.logging_thread = None

        if self.verbose:
            print(
                f"Recording stopped. Collected {len(self.log_data)} samples.")

    def save_log(self, filename: str = None) -> str:
        """
        Save the recorded data to an HDF5 file, ensuring it is fully written to disk.

        Parameters
        ----------
        filename : str, optional
            Path or base name for the HDF5 file. If None, generates a filename
            based on the current timestamp in `self.outpath`.

        Returns
        -------
        str
            Full path of the successfully saved file.

        Raises
        ------
        RuntimeError
            If no data has been recorded or saving fails.
            
        Notes
        -----
        - Data is saved in HDF5 format with metadata
        - File extension '.h5' is automatically added if missing
        - Data includes timestamps, positions, velocities, and forces
        - Files can be loaded later using `load_log()` or external tools
        
        Examples
        --------
        >>> # Save with custom filename
        >>> filepath = robot.save_log("experiment_01")
        >>> print(f"Data saved to: {filepath}")
        >>> 
        >>> # Save with automatic timestamp
        >>> filepath = robot.save_log()
        >>> 
        >>> # Save in specific directory
        >>> filepath = robot.save_log("/data/experiments/trial_05.h5")
        """

        if not self.log_data:
            raise RuntimeError("No data to save — start recording first.")

        # Generate timestamped filename if not provided
        if filename is None:
            filename = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        # Ensure extension is .h5
        if not filename.endswith(".h5"):
            filename += ".h5"

        full_path = os.path.join(self.outpath.rstrip("/"), filename)
        data = np.array(self.log_data)

        try:
            with h5py.File(full_path, "w") as f:
                # Save data
                f.create_dataset("data", data=data)
                f["data"].attrs["columns"] = [
                    "time",
                    "pos_x", "pos_y", "pos_z",
                    "vel_x", "vel_y", "vel_z",
                    "force_x", "force_y", "force_z",
                ]
                # Ensure data is flushed to disk
                f.flush()

            # Verify file existence and non-zero size
            if not os.path.exists(full_path) or os.path.getsize(full_path) == 0:
                raise RuntimeError(
                    f"Failed to fully save log file at {full_path}")

            if self.verbose:
                print(
                    f"Data fully saved to {full_path} ({data.shape[0]} samples).")

            return full_path

        except Exception as e:
            raise RuntimeError(f"Error saving log file: {e}")

    def return_log(self) -> tuple[np.ndarray, list[str]]:
        """
        Return the current recorded data as a NumPy array without saving to file.

        Returns
        -------
        tuple[np.ndarray, list[str]]
            A tuple containing:
            - `data`: 2D NumPy array of recorded samples with shape (n_samples, 10).
            - `columns`: List of column names corresponding to the data.

        Raises
        ------
        RuntimeError
            If no data has been recorded.

        Notes
        -----
        Data columns are:
        ["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "force_x", "force_y", "force_z"]
        
        Example
        -------
        >>> robot.start_recording(freq=1000)
        >>> # ... perform robot operations ...
        >>> robot.stop_recording()
        >>> data, columns = robot.return_log()
        >>> print(f"Recorded {data.shape[0]} samples")
        """
        if not self.log_data:
            raise RuntimeError("No data to return — start recording first.")

        data = np.array(self.log_data)
        columns = [
            "time",
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "force_x", "force_y", "force_z",
        ]

        if self.verbose:
            print(f"Returning {data.shape[0]} recorded samples.")

        return data, columns

    def load_log(self, filename: str = None) -> tuple[np.ndarray, list[str]]:
        """
        Load previously saved robot data from an HDF5 file.

        Parameters
        ----------
        filename : str, optional
            Path to the HDF5 file. If None, defaults to `self.outpath`.

        Returns
        -------
        tuple[np.ndarray, list[str]]
            A tuple containing:
            - `data`: 2D NumPy array of recorded samples.
            - `columns`: List of column names corresponding to the data.

        Raises
        ------
        RuntimeError
            If the file cannot be opened or data cannot be read.
        """

        filename = filename or self.outpath

        if not os.path.exists(filename):
            raise RuntimeError(f"File not found: {filename}")

        try:
            with h5py.File(filename, "r") as f:
                data = np.array(f["data"])
                columns = list(f["data"].attrs.get("columns", []))

            if self.verbose:
                print(f"Loaded {data.shape[0]} samples from {filename}")

            return data, columns

        except Exception as e:
            raise RuntimeError(f"Failed to load log file '{filename}': {e}")

    def plot_log(self, filename: str = None, save: bool = False, save_path: str = None, format: str = "png"):
        """
        Plot position, velocity, and force data from a saved log file.

        Parameters
        ----------
        filename : str, optional
            Path to the HDF5 file. If None, uses `self.outpath`.
        save : bool, default=False
            If True, saves the figure instead of displaying it.
        save_path : str, optional
            Path to save the figure. If None and `save=True`, uses a timestamped file in `self.outpath`.
        format : str, default="png"
            File format for saved figure. Options: "png", "pdf", "svg", "eps", "jpg".
            
            **Optimized format recommendations:**
            - **"pdf"** - Publications/reports (pure vector, embedded fonts, infinite resolution)
            - **"svg"** - Web/interactive (pure vector, scalable, smaller files, web-optimized) 
            - **"eps"** - LaTeX/academic (pure vector PostScript for journals)
            - **"png"** - General use (300 DPI, lossless compression, optimized)
            - **"jpg"** - Web/sharing (300 DPI, 95% quality, progressive, smaller files)
            
        Notes
        -----
        - Creates a 3-panel plot showing position, velocity, and force vs time
        - All three axes (x, y, z) are plotted for each quantity
        - Useful for analyzing robot motion and interaction forces
        - Each format uses optimized settings for maximum quality and efficiency
        - Vector formats (PDF, SVG, EPS) scale infinitely without quality loss
        
        Examples
        --------
        >>> # Display interactive plot
        >>> robot.plot_log("experiment_data.h5")
        >>> 
        >>> # Save as high-quality PDF for publications
        >>> robot.plot_log("data.h5", save=True, save_path="results.pdf", format="pdf")
        >>> 
        >>> # Save as PNG with custom path
        >>> robot.plot_log("data.h5", save=True, save_path="results.png", format="png")
        >>> 
        >>> # Auto-generated filename with PDF format
        >>> robot.plot_log("data.h5", save=True, format="pdf")
        """

        data, columns = self.load_log(filename)
        time = data[:, 0]

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle("Omega Robot Recorded Data", fontsize=14)

        # Position
        axes[0].plot(time, data[:, 1:4])
        axes[0].set_ylabel("Position [m]")
        axes[0].legend(["x", "y", "z"])
        axes[0].grid(True)

        # Velocity
        axes[1].plot(time, data[:, 4:7])
        axes[1].set_ylabel("Velocity [m/s]")
        axes[1].legend(["vx", "vy", "vz"])
        axes[1].grid(True)

        # Force
        axes[2].plot(time, data[:, 7:10])
        axes[2].set_ylabel("Force [N]")
        axes[2].set_xlabel("Time [s]")
        axes[2].legend(["fx", "fy", "fz"])
        axes[2].grid(True)

        plt.tight_layout()

        if save:
            if save_path is None:
                timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                # Use the specified format for auto-generated filename
                save_path = f"{self.outpath.rstrip('/')}/log_plot_{timestamp}.{format}"
            
            # Common options for all formats
            base_options = {
                'bbox_inches': 'tight',
                'edgecolor': 'none', 
                'facecolor': 'white',
                'metadata': {'Creator': 'omega_control'}
            }
            
            # Format-specific additional options
            fmt_lower = format.lower()
            if fmt_lower in ["pdf", "svg", "eps", "ps"]:
                # Vector formats: Pure vector graphics, no DPI needed
                # Backend is auto-selected by matplotlib based on format - no need to specify
                if fmt_lower in ["eps", "ps"]:
                    fmt_lower = 'eps'  # Normalize ps to eps
                plt.savefig(save_path, format=fmt_lower, **base_options)
                
            elif fmt_lower == "png":
                # PNG: High-quality raster with optimization
                plt.savefig(save_path, format='png', dpi=300, transparent=False, 
                           optimize=True, **base_options)
                           
            elif fmt_lower in ["jpg", "jpeg"]:
                # JPEG: Optimized for web/sharing, no transparency
                plt.savefig(save_path, format='jpeg', dpi=300, quality=95, 
                           optimize=True, progressive=True, **base_options)
            else:
                # Fallback for other raster formats
                plt.savefig(save_path, format=format, dpi=300, **base_options)
            
            if self.verbose:
                print(f"Figure saved to {save_path} (format: {format.upper()})")
            plt.close(fig)
        else:
            plt.show()

    # -------------------------------------------------------------------------
    # CLEANUP
    # -------------------------------------------------------------------------

    def close(self):
        """
        Safely stop motion, stop any active logging, and close the robot device connection.
        
        Notes
        -----
        - Always call this method when finished with the robot
        - Automatically stops any active recording
        - Ensures proper cleanup of device resources
        - Should be called in a try/finally block or context manager
        
        Examples
        --------
        >>> # Basic usage
        >>> robot = OmegaRobot()
        >>> # ... use robot ...
        >>> robot.close()
        >>> 
        >>> # Safe usage with try/finally
        >>> robot = OmegaRobot()
        >>> try:
        >>>     robot.start_recording()
        >>>     robot.set_pos([0.05, 0.0, 0.0])
        >>> finally:
        >>>     robot.close()
        """
        # Stop recording if active
        if self.logging_active:
            if self.verbose:
                print("Stopping active recording before closing device...")
            self.stop_recording()

        # Check if device is already closed
        if not self._is_open:
            if self.verbose:
                print("Device is already closed.")
            return

        # Close the device connection
        drd.close(self.device_id)
        self._is_open = False

        if self.verbose:
            print("Robot connection successfully closed.")
