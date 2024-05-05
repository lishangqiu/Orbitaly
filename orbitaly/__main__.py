"""Entry point:  python -m orbitaly [serve|doctor|selftest] [options]"""
from __future__ import annotations

import argparse
import logging
import sys

SUBCOMMANDS = ("serve", "doctor", "selftest")


def build_parser() -> argparse.ArgumentParser:
    # Options every subcommand shares live on a parent parser rather than on
    # the top level, so both "orbitaly -c x" and "orbitaly doctor -c x" work.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--config", "-c", help="Path to config.yaml", default=None)
    common.add_argument("--verbose", "-v", action="store_true")

    parser = argparse.ArgumentParser(prog="orbitaly", description="Antenna tracker ground station")
    subparsers = parser.add_subparsers(dest="command", required=True)

    serve = subparsers.add_parser(
        "serve", parents=[common], help="run the web UI and tracker (default)"
    )
    serve.add_argument("--host", help="Override listen host", default=None)
    serve.add_argument("--port", type=int, help="Override listen port", default=None)

    subparsers.add_parser(
        "doctor", parents=[common], help="report what this machine can drive, and why"
    )

    selftest = subparsers.add_parser(
        "selftest",
        parents=[common],
        help="measure the real pulse train through a loopback jumper",
    )
    from .cli.selftest import add_arguments

    add_arguments(selftest)
    return parser


def normalise_argv(argv: list[str]) -> list[str]:
    """Accept the subcommand anywhere, and default to serving.

    ``orbitaly -c config.yaml`` predates the subcommands and has to keep
    working, and ``orbitaly -c config.yaml doctor`` reads naturally enough
    that rejecting it would just be rude.
    """
    index = None
    for i, arg in enumerate(argv):
        if arg in SUBCOMMANDS and (i == 0 or argv[i - 1] not in ("-c", "--config")):
            index = i
            break
    if index is None:
        return ["serve", *argv]
    return [argv[index], *argv[:index], *argv[index + 1 :]]


def main() -> None:
    parser = build_parser()
    args = parser.parse_args(normalise_argv(sys.argv[1:]))

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    if args.command == "doctor":
        from .cli.doctor import run

        raise SystemExit(run(args.config))

    if args.command == "selftest":
        from .cli.selftest import run

        raise SystemExit(run(args))

    serve(args)


def serve(args: argparse.Namespace) -> None:
    from .app import create_app
    from .config import load_config

    config = load_config(args.config)
    if getattr(args, "host", None):
        config.server.host = args.host
    if getattr(args, "port", None):
        config.server.port = args.port

    import uvicorn

    uvicorn.run(
        create_app(config), host=config.server.host, port=config.server.port, log_level="warning"
    )


if __name__ == "__main__":
    main()
