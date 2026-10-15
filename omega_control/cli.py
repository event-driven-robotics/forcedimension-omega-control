"""CLI for opt-in vendor SDK installation and SDK diagnostics."""
from __future__ import annotations
import argparse
import sys

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="omega-control")
    sub = parser.add_subparsers(dest="command", required=True)
    setup = sub.add_parser("setup-sdk", help="Detect OS and install/download the official vendor SDK")
    setup.add_argument("--yes", action="store_true", help="Approve vendor SDK download noninteractively")
    setup.add_argument("--sdk-path", metavar="PATH", help="Register an existing SDK directory")
    sub.add_parser("doctor", help="Check SDK discovery without requiring a robot")
    args = parser.parse_args(argv)
    if args.command == "doctor":
        from .doctor import main as doctor_main
        return doctor_main()
    from .sdk_installer import SDKSetupError, setup_sdk
    try:
        return setup_sdk(yes=args.yes, sdk_path=args.sdk_path)
    except (SDKSetupError, ValueError, OSError) as exc:
        print(f"SDK setup failed: {exc}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
