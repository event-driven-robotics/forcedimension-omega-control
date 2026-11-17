"""Read and display the Omega end-effector state.

Initialization may move the device. Run only with a supervised, clear
workspace. Device selection uses DRD index plus expected serial, not DHD
serial-only opening. This example sends no Cartesian trajectory.
"""
import argparse
import math
import time

from omega_control import OmegaRobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device-index",
        type=int,
        required=True,
        help="Explicit DRD enumeration index (verify it with device_info_example.py)",
    )
    parser.add_argument(
        "--serial",
        type=int,
        required=True,
        help="Expected hardware serial number (verified before initialization)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="How long to monitor state, in seconds (default: 5)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Delay between displayed samples, in seconds (default: 0.2)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if not math.isfinite(args.interval) or args.interval <= 0:
        parser.error("--interval must be finite and positive")
    return args


def main() -> None:
    args = parse_args()
    robot = OmegaRobot(
        verbose=True,
        auto_open=False,
        auto_initialize=False,
    )

    try:
        robot.open(device_index=args.device_index, expected_serial=args.serial)
        print(f"Connected to serial number {robot.get_serial_number()}")

        # Initialization may move the robot to its reference position.
        robot.initialize()

        control_frequency = robot.get_control_freq()
        if (
            isinstance(control_frequency, (int, float))
            and math.isfinite(control_frequency)
            and control_frequency >= 0
        ):
            print(f"Control-loop frequency: {control_frequency:.2f} kHz")
        else:
            print(f"Control-loop frequency unavailable: {control_frequency!r}")

        # Individual getters are available when only one quantity is needed.
        print("Initial position:", robot.get_pos())
        print("Initial velocity:", robot.get_vel())
        print("Initial force:", robot.get_force())

        # Position reads may legitimately use SDK TimeGuard-cached data.
        # For repeated monitoring, read_checked_state() groups the three
        # checked reads under one SDK lock (not one hardware sample).
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            position, velocity, force = robot.read_checked_state()
            print(
                f"pos [m]={position} | "
                f"vel [m/s]={velocity} | "
                f"force [N]={force}"
            )
            time.sleep(args.interval)
    finally:
        if robot.is_open():
            # This is an SDK BRAKE request, not independent verification of
            # physical braking. Always inspect any reported cleanup failure.
            try:
                robot.stop()
            finally:
                robot.close()


if __name__ == "__main__":
    main()
