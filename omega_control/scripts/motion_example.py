"""Move along Cartesian axes with checked SDK commands and recording.

CAUTION: Initialization and movement can move the device. Supervise the robot
and verify these positions are appropriate for the attached workspace.
"""
from omega_control import OmegaRobot


def main() -> None:
    robot = OmegaRobot(
        verbose=True, auto_open=False, auto_initialize=False
    )
    filename = None
    try:
        robot.open()
        print(f"Connected serial: {robot.get_serial_number()}")
        robot.initialize()  # May move during reference initialization.

        # True means DRD accepted the command, not measured arrival.
        robot.to_origin()  # Raises if the underlying move command is rejected.
        robot.start_recording(freq=1000)
        try:
            for target in (
                [0.02, 0.0, 0.0],
                [0.0, 0.02, 0.0],
                [0.0, 0.0, 0.02],
            ):
                acknowledged = robot.set_pos(target)
                print(f"Move to {target} acknowledged: {acknowledged}")
                robot.check_recording_health()
                robot.to_origin()
        finally:
            # Stop recording even when a command or read fails.
            robot.stop_recording()
        filename = robot.save_log(filename="axis_finding")
    finally:
        if robot.is_open():
            # A failed brake request is reported, not hidden by closing.
            try:
                robot.stop()
            finally:
                robot.close()

    if filename is not None:
        robot.plot_log(filename=filename)


if __name__ == "__main__":
    main()
