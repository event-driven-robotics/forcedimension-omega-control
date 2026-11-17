"""Move along Cartesian axes with checked SDK commands and recording.

CAUTION: Initialization and movement can move the device. Supervise the robot
and verify these positions are appropriate for the attached workspace.
"""
import argparse

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
        print(f"Verified serial: {robot.get_serial_number()}")
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
