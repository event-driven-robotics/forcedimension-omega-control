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
        robot.save_log("session_data.h5")
        robot.close()

Authors:
    Simon Mueller-Cleve (simon.mullercleve@iit.it)
    Quentin Halbach (halbach.quentin@iit.it)

Institution:
    Istituto Italiano di Tecnologia (IIT)

Created:
    2025-10-30

Last Modified:
    2025-10-31
"""

import threading
import os
import time
from datetime import datetime
import forcedimension_core.dhd as dhd
import forcedimension_core.drd as drd
import h5py
import numpy as np
import matplotlib.pyplot as plt


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

        self.default_move_param = self.get_motion_param()
        self.default_track_param = self.get_motion_param(tracking=True)

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
        through the device’s trajectory generation parameters.

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

    def get_motion_param(self, tracking: bool = False):
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

    def preload(self, target_axis, target_force,
                base_step=1e-5, tolerance=0.05, max_iter=5000,
                sleep_time=0.001, max_step=1e-4,
                adapt_gain=1e-4, plot_results=False):
        """
        Gradually apply a preload along a specified axis until a target force is reached.

        This method moves the robot end-effector along a specified Cartesian axis
        using an adaptive step size, based on the difference between the current and
        target force. The process continues until the target force is achieved within
        the specified tolerance or until the maximum number of iterations is reached.

        During execution, the error, applied step size, and measured forces are logged.
        Optionally, these values can be plotted at the end of the procedure.

        Parameters
        ----------
        target_axis : str
            The axis along which to apply the preload ('x', 'y', or 'z').
        target_force : float
            The desired force to reach along the specified axis (in Newtons).
        base_step : float, optional
            Minimum incremental step size along the axis (default: 1e-5 m).
        tolerance : float, optional
            Acceptable deviation from the target force (default: 0.05 N).
        max_iter : int, optional
            Maximum number of iterations to attempt (default: 5000).
        sleep_time : float, optional
            Delay in seconds between iterations (default: 0.001 s).
        max_step : float, optional
            Maximum allowed step size along the axis (default: 1e-4 m).
        adapt_gain : float, optional
            Gain factor for adaptive step scaling based on force error (default: 1e-4).
        plot_results : bool, optional
            If True, generates a plot of force error, steps, and measured forces at the end (default: False).

        Returns
        -------
        float or bool
            - The final position along the specified axis if the target force is reached.
            - False if the maximum number of iterations is reached without achieving the target force.

        Notes
        -----
        - The method temporarily overrides the current trajectory parameters and restores
        the previous motion parameters upon completion.
        - Force measurements must be available from the device for the preload to function.
        - Adaptive step size is clipped between `base_step` and `max_step` to ensure safe motion.

        Examples
        --------
        >>> final_pos = robot.preload('z', target_force=5.0, plot_results=True)
        >>> if final_pos:
        >>>     print(f"Preload achieved at z={final_pos:.4f} m")
        """

        self.get_pos()
        current_pos = self.pos.copy()
        axis_list = ["x", "y", "z"]
        axis_idx = axis_list.index(target_axis)

        move_params = self.get_pos_param()

        self.set_default_pos_param()

        # Storage for analysis
        errors = []
        steps = []
        forces = []

        for i in range(max_iter):
            self.get_force()
            current_force = self.force[axis_idx]
            error = target_force - current_force

            errors.append(error)
            forces.append(current_force)

            if abs(error) <= tolerance:
                if self.verbose:
                    print(
                        f"Target force reached: {current_force:.4f} N (iteration {i})")
                if plot_results:
                    self._plot_preload_results(
                        errors, steps, forces, target_force)

                self.set_pos_param(
                    vmax=move_params[0], amax=move_params[1], jerk=move_params[2])
                return self.get_pos()[axis_idx]

            # Direction: positive if need to increase force, negative if need to decrease
            direction = np.sign(error)

            # Adaptive step: proportional to |error|
            adaptive_step = base_step + adapt_gain * abs(error)
            adaptive_step = np.clip(adaptive_step, base_step, max_step)
            steps.append(adaptive_step)

            # Move along axis
            current_pos[axis_idx] += direction * adaptive_step
            self.set_pos(current_pos, block=True)

            if self.verbose:
                print(
                    f"[Iter {i}] Force={current_force:.4f} N | Error={error:.4f} | Step={adaptive_step:.6f}")

            time.sleep(sleep_time)

        if self.verbose:
            print(
                f"Preload stopped after {max_iter} steps. Final force: {self.force[axis_idx]:.4f} N")

        if plot_results:
            self._plot_preload_results(errors, steps, forces, target_force)

        return False

    def _plot_preload_results(self, errors, steps, forces, target_force, save=False, filename=None):
        """
        Plot convergence data from the preload routine.

        Displays force error and adaptive step size over iterations,
        allowing visualization of how the robot approaches the target force.

        Parameters
        ----------
        errors : list of float
            Force errors at each iteration (target_force - measured_force).
        steps : list of float
            Step sizes applied along the axis at each iteration.
        forces : list of float
            Measured force along the preload axis at each iteration.
        target_force : float
            The target force applied along the axis.
        save : bool, optional
            If True, saves the figure to a file instead of displaying it (default: False).
        filename : str, optional
            Path to save the figure. If None, defaults to "preload_convergence.png".
        """

        iterations = range(len(errors))

        fig, ax1 = plt.subplots(figsize=(8, 6))

        # Plot force error
        color1 = 'tab:blue'
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Force Error (N)', color=color1)
        ax1.plot(iterations, errors, color=color1,
                 label='Force Error', linewidth=1.5)
        ax1.axhline(0, color='gray', linestyle='--', linewidth=1)
        ax1.tick_params(axis='y', labelcolor=color1)
        ax1.grid(True, linestyle='--', alpha=0.5)

        # Plot adaptive step size on secondary y-axis
        ax2 = ax1.twinx()
        color2 = 'tab:red'
        ax2.set_ylabel('Step Size (m)', color=color2)
        ax2.plot(iterations[:len(steps)], steps,
                 color=color2, label='Step Size', linewidth=1.5)
        ax2.tick_params(axis='y', labelcolor=color2)

        # Optional: add legend combining both axes
        lines, labels = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines + lines2, labels + labels2, loc='upper right')

        plt.title(
            f"Adaptive Preload Convergence (Target Force = {target_force:.3f} N)")
        fig.tight_layout()

        if save:
            if filename is None:
                filename = "preload_convergence.png"
            plt.savefig(filename, dpi=300)
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

    def plot_log(self, filename: str = None, save: bool = False, save_path: str = None):
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
                save_path = f"{self.outpath.rstrip('/')}/log_plot_{timestamp}.png"
            plt.savefig(save_path)
            if self.verbose:
                print(f"Figure saved to {save_path}")
            plt.close(fig)
        else:
            plt.show()

    # -------------------------------------------------------------------------
    # CLEANUP
    # -------------------------------------------------------------------------

    def close(self):
        """
        Safely stop motion, stop any active logging, and close the robot device connection.
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
