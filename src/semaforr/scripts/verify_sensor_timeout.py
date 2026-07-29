#!/usr/bin/env python3

"""Verify the live stale-sensor safe-stop trace produced by the recorder."""

import argparse
import json
from pathlib import Path


def is_zero(command):
    return all(
        command[field] == 0.0
        for field in ("linear_x", "linear_y", "angular_z")
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()

    trace = json.loads(args.trace.read_text(encoding="utf-8"))
    result = trace["result"]
    commands = result.get("commands", [])
    states = result.get("navigation_states", [])

    failures = []
    if trace["scenario"].get("sensor_cutoff_s") is None:
        failures.append("scenario did not enable a sensor cutoff")
    if not commands or not any(not is_zero(command) for command in commands):
        failures.append("trace did not execute a non-zero command")
    if not commands or not is_zero(commands[-1]):
        failures.append("last velocity command is not zero")
    if not any(
        state.get("state") == "waiting_for_sensors"
        and state.get("detail", "").startswith("sensor_")
        and state.get("detail", "").endswith("_stale")
        and state.get("failure")
        for state in states
    ):
        failures.append("no stale-sensor transition was recorded")

    if failures:
        for failure in failures:
            print(f"sensor-timeout verification failed: {failure}")
        return 1

    print("SemaFORR stopped safely after sensor loss")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
