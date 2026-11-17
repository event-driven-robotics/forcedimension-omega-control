"""Inspect an Omega device identity without initializing or commanding motion.

Run with:
    python omega_control/scripts/device_info_example.py
    python omega_control/scripts/device_info_example.py --device-index 0
    python omega_control/scripts/device_info_example.py --serial 123456

Opening and reading a serial number do not call DRD initialization. The script
does not issue motion commands. Verify the vendor SDK's open behavior for your
hardware before relying on this as a motion-free operation.
"""
import argparse

from omega_control import OmegaRobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device-index",
        type=int,
        help="DRD enumeration index to open (default: SDK-selected device)",
    )
    parser.add_argument(
        "--serial",
        type=int,
        help=(
            "Expected hardware serial number. With --device-index, verify that "
            "device; without it, open the device by serial number."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = OmegaRobot(
        verbose=True,
        auto_open=False,
        auto_initialize=False,
    )

    try:
        device_id = robot.open(
            device_index=args.device_index,
            expected_serial=args.serial,
        )
        serial_number = robot.get_serial_number()

        print(f"Connection open: {robot.is_open()}")
        print(f"SDK device ID: {device_id}")
        print(f"Hardware serial number: {serial_number}")
        print("Initialization was not requested.")
    finally:
        # If identity verification failed and its first cleanup attempt also
        # failed, make one more explicit close attempt.
        if robot.is_open():
            robot.close()


if __name__ == "__main__":
    main()
