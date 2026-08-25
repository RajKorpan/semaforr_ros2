#!/usr/bin/env python3

"""SemaFORR module overview.

Summary:
    This file implements verify sensor timeout behavior for developer tooling and experiment automation. It centers on `is_zero`, `main`. Its package-relative location is `scripts/verify_sensor_timeout.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""










import argparse
import json
from pathlib import Path


def is_zero(command):
    """Summary:
        Reports whether zero for this subsystem.

    Args:
        command (Any): Supplies command input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    return all(
        command[field] == 0.0
        for field in ("linear_x", "linear_y", "angular_z")
    )


def main():
    """Summary:
        Performs the main operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
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
