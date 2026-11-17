"""Acquire and inspect a bounded force preload.

CAUTION: Requires a supervised, known contact geometry. Initialization and
preload control may move the device; tune the sign and gains for your setup.
"""
import argparse

from omega_control import OmegaRobot, PreloadStatus


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
        print(f"Verified device serial: {robot.get_serial_number()}")
        robot.initialize()
        robot.to_origin()
        robot.start_recording(freq=1000)
        try:
            # Starting pose and force direction are hardware/setup dependent.
            robot.set_pos([0.0, 0.0, -0.052])

            result = robot.preload(
                target_axis="z",
                target_force=-1.0,
                kp=1e-3,
                ki=0.0,
                max_force=2.0,
                max_displacement=0.01,
                plot_results=False,
            )
            print(
                f"Preload status={result.status.value}, "
                f"measured force={result.final_force:.3f} N, "
                f"position={result.final_position:.4f} m"
            )
            if result.status is PreloadStatus.SETTLED:
                print("Target preload settled within the requested tolerance.")
            else:
                print(f"Preload was not successful: {result.message}")
            robot.check_recording_health()
        finally:
            robot.stop_recording()

        filename = robot.save_log(filename="preload_data")
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
