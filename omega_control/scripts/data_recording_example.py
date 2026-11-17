"""Record Omega position, velocity and force, then inspect and save samples.

CAUTION: Initialization may move the device. A requested acquisition rate is
not a guarantee of deterministic hardware sampling. The stored time column is
elapsed wall-clock time, not a synchronized device timestamp.
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
    robot = OmegaRobot(
        verbose=True, auto_open=False, auto_initialize=False
    )
    filename = None
    try:
        robot.open(device_index=args.device_index, expected_serial=args.serial)
        print(f"Recording verified device serial: {robot.get_serial_number()}")
        robot.initialize()  # May move the device.

        robot.start_recording(freq=1000)
        try:
            for _ in range(20):
                time.sleep(0.1)
                # Propagate failures from the recorder's background thread.
                robot.check_recording_health()
        finally:
            robot.stop_recording()

        # Inspect data directly, without writing a file first.
        data, columns = robot.return_log()
        print(f"Recorded {data.shape[0]} samples, with columns: {columns}")

        # save_log refuses incomplete recordings after acquisition failure.
        filename = robot.save_log(filename="recording_example")
        print(f"Saved to: {filename}")
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
