"""Repeated Cartesian Y-axis motion with checked commands.

CAUTION: This script requests repeated 9 cm motion. Confirm the configured
travel is safe for your hardware and workspace before running. Initialize
only after selecting an explicit DRD index and verifying the device serial.
"""
import argparse
import time

from omega_control import OmegaRobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-index", type=int, required=True)
    parser.add_argument("--serial", type=int, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    repetitions = 50
    amplitude = 0.09  # 9 cm
    vmax = 0.1  # m/s
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
            # Motion and tracking parameters are configured separately.
            robot.set_pos_param(vmax, 1.0, 1.0)
            try:
                for _ in range(repetitions):
                    robot.set_pos([0.0, amplitude, 0.0])
                    robot.set_pos([0.0, -amplitude, 0.0])
                    robot.check_recording_health()
            except KeyboardInterrupt:
                print("Interrupted; terminating recording and closing.")
        finally:
            try:
                robot.stop_recording()
            finally:
                robot.set_default_pos_param()

        filename = robot.save_log(filename="line_fast_robot")
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
