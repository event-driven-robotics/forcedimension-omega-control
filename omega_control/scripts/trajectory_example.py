"""Sinusoidal Cartesian tracking trajectory.

CAUTION: This script commands motion. Use a clear, supervised workspace and
an independently accessible emergency stop. Identity is verified before
moving initialization is requested.
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
    amplitude = 0.03  # 3 cm base amplitude
    frequency = 0.25  # Hz
    duration = 5.0  # seconds
    filename = None
    robot = OmegaRobot(verbose=False, auto_open=False, auto_initialize=False)

    try:
        robot.open(device_index=args.device_index, expected_serial=args.serial)
        print(f"Verified serial: {robot.get_serial_number()}")
        robot.initialize()
        robot.to_origin()

        robot.start_recording(freq=1000)
        try:
            start = time.monotonic()
            try:
                while (t := time.monotonic() - start) < duration:
                    phase = 2 * math.pi * frequency * t
                    x = amplitude * math.sin(phase)
                    y = 1.5 * amplitude * math.sin(phase)
                    z = 0.5 * amplitude * math.sin(phase)
                    robot.set_pos([x, y, z], tracking=True)
                    time.sleep(0.001)  # Best effort, not deterministic 1 kHz.
            except KeyboardInterrupt:
                print("Tracking interrupted; closing the robot.")
        finally:
            robot.stop_recording()

        filename = robot.save_log(filename="trajectory")
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
