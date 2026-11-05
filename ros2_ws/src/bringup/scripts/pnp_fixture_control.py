#!/usr/bin/env python3
"""Validation-private command client for an already-running Isaac session."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time


def _load_protocol():
    root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not root:
        raise RuntimeError("SFTWIN_PROJECT_ROOT is required")
    module_root = Path(root) / "infra/isaac_sim/scripts/m0609_cell"
    sys.path.insert(0, str(module_root))
    from pnp_validation.live_fixture import FixtureCommandWriter, FixtureResponseReader

    return FixtureCommandWriter, FixtureResponseReader


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "operation",
        choices=("spawn_fixture", "remove_fixture", "query_fixture", "respawn_fixture"),
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--fixture-id", required=True)
    parser.add_argument(
        "--command", default="/tmp/sftwin_pnp_validation/fixture_command.json"
    )
    parser.add_argument(
        "--response", default="/tmp/sftwin_pnp_validation/fixture_response.json"
    )
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--candidate-id")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    args = parser.parse_args(argv)
    Writer, Reader = _load_protocol()
    payload = {"seed": args.seed, "index": args.index}
    if args.candidate_id:
        payload["candidate_id"] = args.candidate_id
    command = Writer(
        args.command, run_id=args.run_id, fixture_id=args.fixture_id
    ).write(args.operation, payload)
    reader = Reader(args.response)
    deadline = time.monotonic() + args.timeout_s
    while time.monotonic() < deadline:
        try:
            response = reader.read_for(command)
        except (FileNotFoundError, ValueError, json.JSONDecodeError):
            time.sleep(0.05)
            continue
        print(json.dumps(response, indent=2, sort_keys=True))
        return 0 if response.get("status") == "OK" else 2
    raise RuntimeError("timed out waiting for correlated Isaac fixture response")


if __name__ == "__main__":
    raise SystemExit(main())
