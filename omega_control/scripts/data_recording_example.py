"""Record Omega position, velocity and force, then inspect and save samples.

CAUTION: Initialization may move the device. A requested acquisition rate is
not a guarantee of deterministic hardware sampling. The stored time column is
elapsed wall-clock time, not a synchronized device timestamp.
"""
import time

from omega_control import OmegaRobot


def main() -> None:
    robot = OmegaRobot(
        verbose=True, auto_open=False, auto_initialize=False
    )
    filename = None
    try:
        robot.open()
        print(f"Recording device serial: {robot.get_serial_number()}")
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
