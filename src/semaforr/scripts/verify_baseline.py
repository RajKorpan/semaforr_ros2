#!/usr/bin/env python3

"""Compare two runtime traces while ignoring nondeterministic timestamps."""

import argparse
import json
from pathlib import Path
import sys


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def command_signature(trace):
    section = trace["expected"] if "expected" in trace else trace["result"]
    commands = section["commands"]
    return [
        (
            event["linear_x"],
            event["linear_y"],
            event["angular_z"],
        )
        for event in commands
    ]


def decision_signature(trace):
    section = trace["expected"] if "expected" in trace else trace["result"]
    decisions = section["decisions"]
    keys = (
        "task",
        "decision",
        "target",
        "max_forward_parameter",
        "decision_tier",
        "chosen_action",
        "chosen_planner",
    )
    return [tuple(json.dumps(event[key], sort_keys=True) for key in keys)
            for event in decisions]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("expected", type=Path)
    parser.add_argument("actual", type=Path)
    args = parser.parse_args()

    expected = load(args.expected)
    actual = load(args.actual)

    differences = []
    expected_scenario = expected["scenario"]
    actual_scenario = actual["scenario"]
    if expected_scenario != actual_scenario:
        differences.append("scenario metadata differs")
    if command_signature(expected) != command_signature(actual):
        differences.append("velocity command sequence differs")
    if decision_signature(expected) != decision_signature(actual):
        differences.append("semantic decision sequence differs")

    if differences:
        for difference in differences:
            print(f"baseline mismatch: {difference}", file=sys.stderr)
        return 1

    print("SemaFORR runtime trace matches the approved baseline")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
