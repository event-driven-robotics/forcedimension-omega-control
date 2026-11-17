"""Cartesian circular tracking trajectory.

CAUTION: This script commands motion. Confirm the circular path is within
your known safe workspace, and verify device identity before initialization.
"""
import argparse
import math
import time

from omega_control import OmegaRobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-index", type=int, required=True)
    parser.add_argument("--serial", type=int, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    radius = 0.04  # 4 cm
    frequency = 0.25  # Hz
    duration = 60.0  # seconds
    filename = None
    robot = OmegaRobot(verbose=False, auto_open=False, auto_initialize=False)

    try:
        robot.open(device_index=args.device_index, expected_serial=args.serial)
        print(f"Verified serial: {robot.get_serial_number()}")
        robot.initialize()

        # Move to the starting point before streaming tracking commands.
        robot.set_pos([radius, 0.0, 0.0])
        time.sleep(0.5)
        robot.start_recording(freq=1000)
        try:
            start = time.monotonic()
            omega = 2 * math.pi * frequency
            try:
                while (t := time.monotonic() - start) < duration:
                    x = radius * math.cos(omega * t)
                    y = radius * math.sin(omega * t)
                    robot.set_pos([x, y, 0.0], tracking=True)
                    time.sleep(0.001)  # Requested loop cadence, not guaranteed.
            except KeyboardInterrupt:
                print("Tracking interrupted; closing the robot.")
        finally:
            robot.stop_recording()

        filename = robot.save_log(filename="circle_slow")
    finally:
        if robot.is_open():
            try:
                robot.stop()
            finally:
                robot.close()

    if filename is not None:
        robot.plot_log(filename=filename)


if __name__ == "__main__":
    main()
