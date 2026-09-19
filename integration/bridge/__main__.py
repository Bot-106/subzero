"""`python -m bridge --server 127.0.0.1 --profile mock` — Subzero NT4 <-> tool HTTP bridge."""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from pathlib import Path

from .core import Bridge, load_config
from .nt_client import NtClient

DEFAULT_CONFIG = Path(__file__).with_name("config.yaml")


def parse_args(argv=None):
    p = argparse.ArgumentParser(prog="bridge", description=__doc__)
    p.add_argument("--server", default=None, help="NT4 server host: 127.0.0.1 (sim) or 10.13.60.2 (RoboRIO); default from profile")
    p.add_argument("--port", type=int, default=0, help="NT4 port (0 = default 5810)")
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--profile", choices=["real", "mock"], default="mock")
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args(argv)


async def amain(args) -> None:
    tools, timings, prof = load_config(args.config, args.profile)
    server = args.server or prof.get("nt_server") or "127.0.0.1"
    nt = NtClient(server=server, port=args.port)
    bridge = Bridge(nt, tools, timings)
    logging.getLogger("bridge").info("profile=%s server=%s tools=%s timings=%s", args.profile, server, tools, timings)
    await bridge.run()


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        logging.getLogger("bridge").info("Ctrl-C: shut down cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
