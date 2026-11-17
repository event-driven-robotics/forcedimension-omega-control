"""Cartesian figure-eight tracking trajectory.

CAUTION: This script commands motion. Confirm the trajectory fits the
workspace and verify device identity before moving initialization.
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
    amplitude = 0.03  # 3 cm
    frequency = 0.25  # Hz
    duration = 25.0  # seconds
    filename = None
    robot = OmegaRobot(verbose=False, auto_open=False, auto_initialize=False)

    try:
        robot.open(device_index=args.device_index, expected_serial=args.serial)
        print(f"Verified serial: {robot.get_serial_number()}")
        robot.initialize()
        robot.to_origin()
        time.sleep(0.5)

        robot.start_recording(freq=1000)
        try:
            start = time.monotonic()
            omega = 2 * math.pi * frequency
            try:
                while (t := time.monotonic() - start) < duration:
                    x = amplitude * math.sin(2 * omega * t)
                    y = amplitude * math.sin(omega * t)
                    robot.set_pos([x, y, 0.0], tracking=True)
                    time.sleep(0.001)  # Requested loop cadence, not guaranteed.
            except KeyboardInterrupt:
                print("Tracking interrupted; closing the robot.")
        finally:
            robot.stop_recording()

        filename = robot.save_log(filename="figure8")
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
