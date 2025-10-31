"""
Provides a unified interface for controlling and reading the state
of a Force Dimension Omega robot via the DHD/DRD APIs, including
threaded data recording capabilities.

Authors:
  Simon Mueller-Cleve (simon.mullercleve@iit.it)
  Quentin Halbach (halbach.quentin@iit.it)

Created at Istituto Italiano di Tecnologia (IIT)
Last updated: 2023-10-05

"""


"""
Provides a unified interface for controlling and reading the state
of a Force Dimension Omega robot via the DHD/DRD APIs, including
threaded data recording capabilities.
"""

import os
import time
import threading
import numpy as np
import h5py
import forcedimension_core.drd as drd
import forcedimension_core.dhd as dhd
from datetime import datetime
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
        """Open a connection to the Force Dimension device if not already open."""
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
        """Return True if the device connection is open."""
        return self._is_open
    
    def force_init(self):
        """Force re-initialization of the Force Dimension device."""
        drd.autoInit(self.device_id)

    def initialize(self):
        """Initialize or start the Force Dimension device."""
        try:
            if not drd.isInitialized():
                drd.autoInit(self.device_id)
            else:
                drd.start()
            if self.verbose:
                print("Robot successfully initialized.")
        except Exception as e:
            drd.close(self.device_id)
            raise RuntimeError(f"Failed to initialize the robot: {e}")

    def restart(self):
        """Restart the DRD control loop."""
        drd.start(self.device_id)
        if self.verbose:
            print("Device restarted.")

    def to_origin(self):
        self.set_pos([0.0, 0.0, 0.0])

    def get_pos(self):
        """Query and update the current position of the device."""
        dhd.getPosition(self.pos)
        return self.pos

    def get_vel(self):
        """Query and update the current velocity of the device."""
        dhd.getLinearVelocity(self.vel)
        return self.vel

    def get_force(self):
        """Query and update the current force measured by the device."""
        dhd.getForce(self.force)
        return self.force

    def set_pos(self, target, block=True, tracking=False):
        """Move the device to a specified Cartesian position."""
        if not tracking:
            drd.moveToPos(target, block=block)
            if self.verbose:
                print(f"Moving to {target} (block={block})")
        else:
            drd.trackPos(target)
            if self.verbose:
                print(f"Tracking to {target}.")
    
    def get_pos_param(self, tracking=False):
        """Sets the cartesian motion trajectory generation
        parameters."""
    
        if not tracking:
            params = drd.getPosMoveParam(self.device_id)
            if self.verbose:
                print(f"Current position params to vmax={params[0]}, amax={params[1]}, jerk={params[2]} for move.")
        else:
            params = drd.getPosTrackParam(self.device_id)
            if self.verbose:
                print(f"Current position params to vmax={params[0]}, amax={params[1]}, jerk={params[2]} for tracking.")
        return params
    
    def set_pos_param(self, vmax, amax, jerk, tracking=False):
        """Sets the cartesian motion trajectory generation
        parameters."""
    
        if not tracking:
            drd.setPosMoveParam(vmax, amax, jerk, self.device_id)
            if self.verbose:
                print(f"Setting position params to vmax={vmax}, amax={amax}, jerk={jerk} for move.")
        else:
            drd.setPosTrackParam(vmax, amax, jerk, self.device_id)
            if self.verbose:
                print(f"Setting position params to vmax={vmax}, amax={amax}, jerk={jerk} for tracking.")
    
    def set_default_pos_param(self, tracking=False):
        """Sets the cartesian motion trajectory generation
        parameters to default values."""
    
        if not tracking:
            self.set_pos_param(vmax=self.default_move_param[0], amax=self.default_move_param[1], jerk=self.default_move_param[2], tracking=False)
            if self.verbose:
                print(f"Setting position params to default for move.")
        else:
            self.set_pos_param(vmax=self.default_track_param[0], amax=self.default_track_param[1], jerk=self.default_track_param[2], tracking=True)
            if self.verbose:
                print(f"Setting position params to default for tracking.")

        return 
    
    def set_force(self, target):
        """Apply a Cartesian force to the device end-effector."""
        dhd.setForce(target)
        if self.verbose:
            print(f"Setting target force to {target}.")

    def preload(self, target_axis, target_force, 
            base_step=1e-5, tolerance=0.05, max_iter=5000, 
            sleep_time=0.001, max_step=1e-4, 
            adapt_gain=1e-4, plot_results=False):
        """
        Gradually apply a preload along a specified axis until a target force is reached,
        using an adaptive step size based on the difference between current and target force.
        Logs the error and step size, and optionally plots them when finished.
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
                    print(f"Target force reached: {current_force:.4f} N (iteration {i})")
                if plot_results:
                    self._plot_preload_results(errors, steps, forces, target_force)
                
                self.set_pos_param(vmax=move_params[0], amax=move_params[1], jerk=move_params[2])
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
                print(f"[Iter {i}] Force={current_force:.4f} N | Error={error:.4f} | Step={adaptive_step:.6f}")

            time.sleep(sleep_time)

        if self.verbose:
            print(f"Preload stopped after {max_iter} steps. Final force: {self.force[axis_idx]:.4f} N")

        if plot_results:
            self._plot_preload_results(errors, steps, forces, target_force)

        return False
    
    def set_vibration(self, amplitude, frequency):
        """
        Set a vibration pattern on the device.

        Parameters
        ----------
        amplitude : float
            Amplitude of the vibration in Newtons.
        frequency : float
            Frequency of the vibration in Hertz.
        """
        dhd.setVibration(amplitude, frequency, ID=self.device_id)
        if self.verbose:
            print(f"Setting vibration: amplitude={amplitude} N, frequency={frequency} Hz")


    def _plot_preload_results(self, errors, steps, forces, target_force):
        """Helper function to plot preload convergence data."""
        iterations = range(len(errors))
        fig, ax1 = plt.subplots(figsize=(8, 6))

        color1 = 'tab:blue'
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Force Error (N)', color=color1)
        ax1.plot(iterations, errors, color=color1, label='Force Error')
        ax1.tick_params(axis='y', labelcolor=color1)
        ax1.axhline(0, color='gray', linestyle='--', linewidth=1)

        ax2 = ax1.twinx()  # share x-axis for step size
        color2 = 'tab:red'
        ax2.set_ylabel('Step Size (m)', color=color2)
        ax2.plot(iterations[:len(steps)], steps, color=color2, label='Step Size')
        ax2.tick_params(axis='y', labelcolor=color2)

        fig.tight_layout()
        plt.title("Adaptive Preload Convergence")
        plt.show()

    # -------------------------------------------------------------------------
    # DATA RECORDING METHODS
    # -------------------------------------------------------------------------

    def _record_loop(self):
        """Private thread function to record data continuously."""
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
                    [now - t0] + self.pos + self.vel + self.force
                )
                last_time = now

            if self.log_freq:
                # Sleep small amount to maintain frequency
                sleep_time = max(0.0, interval - (time.time() - last_time))
                if sleep_time > 0:
                    time.sleep(sleep_time)

        if self.verbose:
            print(f"Recording stopped. Collected {len(self.log_data)} samples.")

    def start_recording(self, freq=None):
        """
        Start recording position, velocity, and force data in a background thread.

        Parameters
        ----------
        freq : float, optional
            Sampling frequency in Hz. If None, runs as fast as possible.
        """
        if self.logging_active:
            if self.verbose:
                print("Recording already active.")
            return

        self.log_freq = freq
        self.log_data = []
        self.logging_active = True
        self.logging_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.logging_thread.start()

    def stop_recording(self):
        """
        Stop the background data recording thread.
        """
        if not self.logging_active:
            if self.verbose:
                print("No active recording to stop.")
            return

        self.logging_active = False
        self.logging_thread.join()
        self.logging_thread = None

    def save_log(self, filename=None):
        """
        Save recorded data to an HDF5 file, ensuring it is fully written to disk
        before returning.

        Parameters
        ----------
        filename : str, optional
            Path or base name for the HDF5 file. If None, uses timestamp in self.outpath.

        Returns
        -------
        str
            Full path of the successfully saved file.

        Raises
        ------
        RuntimeError
            If no data is recorded or saving fails.
        """
        if not self.log_data:
            raise RuntimeError("No data to save — start recording first.")

        if filename is None:
            filename = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        if filename.endswith(".h5"):
            filename = filename[:-3]
        full_path = f"{self.outpath.rstrip('/')}/{filename}.h5"

        data = np.array(self.log_data)

        try:
            with h5py.File(full_path, "w") as f:
                f.create_dataset("data", data=data)
                f["data"].attrs["columns"] = [
                    "time",
                    "pos_x", "pos_y", "pos_z",
                    "vel_x", "vel_y", "vel_z",
                    "force_x", "force_y", "force_z",
                ]
                # Explicitly flush HDF5 buffers
                f.flush()
                # os.fsync(f.fileno())  # Force filesystem sync

            # Verify that file exists and is non-empty
            if not os.path.exists(full_path) or os.path.getsize(full_path) == 0:
                raise RuntimeError(f"Failed to fully save log file at {full_path}")

            if self.verbose:
                print(f"Data fully saved to {full_path} ({data.shape[0]} samples).")

            return full_path

        except Exception as e:
            raise RuntimeError(f"Error saving log file: {e}")
        
    def load_log(self, filename=None):
        """
        Load previously saved robot data from an HDF5 file.

        Parameters
        ----------
        filename : str, optional
            Path to the HDF5 file. If None, uses self.outpath.

        Returns
        -------
        tuple[np.ndarray, list[str]]
            The data array and corresponding column labels.
        """
        filename = filename or self.outpath

        with h5py.File(filename, "r") as f:
            data = np.array(f["data"])
            columns = list(f["data"].attrs["columns"])
        if self.verbose:
            print(f"Loaded {data.shape[0]} samples from {filename}")

        return data, columns


    def plot_log(self, filename=None):
        """
        Plot position, velocity, and force data from a saved log file.

        Parameters
        ----------
        filename : str, optional
            Path to the HDF5 file. If None, uses self.outpath.
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
        plt.show()

    # -------------------------------------------------------------------------
    # CLEANUP
    # -------------------------------------------------------------------------

    def close(self):
        """Safely stop motion, stop logging, and close the device connection."""
        if self.logging_active:
            self.stop_recording()

        if not self._is_open:
            if self.verbose:
                print("Device already closed.")
            return

        drd.close(self.device_id)
        self._is_open = False
        if self.verbose:
            print("Robot connection closed.")
