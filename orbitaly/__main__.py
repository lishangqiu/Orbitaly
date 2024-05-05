"""Entry point:  python -m orbitaly [--config config.yaml]"""
from __future__ import annotations

import argparse
import logging


def main() -> None:
    parser = argparse.ArgumentParser(prog="orbitaly", description="Antenna tracker ground station")
    parser.add_argument("--config", "-c", help="Path to config.yaml", default=None)
    parser.add_argument("--host", help="Override listen host", default=None)
    parser.add_argument("--port", type=int, help="Override listen port", default=None)
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    from .app import create_app
    from .config import load_config

    config = load_config(args.config)
    if args.host:
        config.server.host = args.host
    if args.port:
        config.server.port = args.port

    import uvicorn

    uvicorn.run(create_app(config), host=config.server.host, port=config.server.port, log_level="warning")


if __name__ == "__main__":
    main()
