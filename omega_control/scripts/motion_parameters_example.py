"""Inspect, change, and restore Cartesian motion/tracking limits.

Initialization may move the device. The example changes trajectory-generator
limits but does not issue a Cartesian movement command. Choose limits suitable
for the connected device, end-effector, and workspace.
"""
import argparse

from omega_control import OmegaRobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vmax", type=float, default=0.05, help="Velocity limit in m/s")
    parser.add_argument(
        "--amax", type=float, default=0.1, help="Acceleration limit in m/s^2"
    )
    parser.add_argument(
        "--jerk", type=float, default=0.5, help="Jerk limit in m/s^3"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = OmegaRobot(verbose=True)

    try:
        original_move = robot.get_pos_param()
        original_track = robot.get_pos_param(tracking=True)

        print("Original move parameters (vmax, amax, jerk, status):", original_move)
        print("Original tracking parameters (vmax, amax, jerk, status):", original_track)

        # The same illustrative limits are applied separately to move and
        # tracking modes. set_pos_param() checks the SDK status and readback.
        print(
            "Move parameters accepted:",
            robot.set_pos_param(args.vmax, args.amax, args.jerk),
        )
        print(
            "Tracking parameters accepted:",
            robot.set_pos_param(
                args.vmax,
                args.amax,
                args.jerk,
                tracking=True,
            ),
        )

        print("Updated move parameters:", robot.get_pos_param())
        print("Updated tracking parameters:", robot.get_pos_param(tracking=True))
    finally:
        # Restore the values captured during initialization, even if changing
        # one of the parameter sets fails.
        try:
            robot.set_default_pos_param()
        finally:
            try:
                robot.set_default_pos_param(tracking=True)
            finally:
                robot.close()


if __name__ == "__main__":
    main()
