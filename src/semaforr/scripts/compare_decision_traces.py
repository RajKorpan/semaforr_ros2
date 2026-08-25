#!/usr/bin/env python3
"""SemaFORR module overview.

Summary:
    This file implements compare decision traces behavior for developer tooling and experiment automation. It centers on `_decisions`, `compare`, `main`. Its package-relative location is `scripts/compare_decision_traces.py`.

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
from typing import Any


SEMANTIC_FIELDS = (
    "chosen_action",
    "decision_tier",
    "chosen_planner",
)
ALLOWED_DIFFERENCES = {
    "acceptable_intentional_improvement",
    "regression_requiring_correction",
}


def _decisions(document: dict[str, Any]) -> list[dict[str, Any]]:
    """Summary:
        Performs the decisions operation for this subsystem.

    Args:
        document (dict[str, Any]): Supplies document input to the operation.

    Returns:
        list[dict[str, Any]]

    Raises:
        ValueError: If required input or state is invalid.
    """
    section = document.get("expected", document.get("result", document))
    decisions = section.get("decisions")
    if not isinstance(decisions, list):
        raise ValueError("trace must contain a decisions array")
    return decisions


def compare(
    baseline: dict[str, Any],
    current: dict[str, Any],
    annotations: dict[str, Any],
) -> dict[str, Any]:
    """Summary:
        Return a complete, deterministic classification report.

    Args:
        baseline (dict[str, Any]): Supplies baseline input to the operation.
        current (dict[str, Any]): Supplies current input to the operation.
        annotations (dict[str, Any]): Supplies annotations input to the operation.

    Returns:
        dict[str, Any]

    Raises:
        None documented; dependency failures may propagate.
    """
    expected = {
        f"{decision['task']}:{decision['decision']}": decision
        for decision in _decisions(baseline)
    }
    actual = {
        f"{decision['task']}:{decision['decision']}": decision
        for decision in _decisions(current)
    }
    classified: list[dict[str, Any]] = []
    errors: list[str] = []

    for key in sorted(set(expected) | set(actual)):
        before = expected.get(key)
        after = actual.get(key)
        differences = []
        if before is None or after is None:
            differences.append("decision_presence")
        else:
            differences = [
                field
                for field in SEMANTIC_FIELDS
                if before.get(field) != after.get(field)
            ]

        if not differences:
            classified.append(
                {"decision": key, "classification": "exact_match"})
            continue

        annotation = annotations.get(key)
        if not isinstance(annotation, dict):
            errors.append(f"{key}: semantic difference is unclassified")
            continue
        classification = annotation.get("classification")
        rationale = annotation.get("rationale", "").strip()
        if classification not in ALLOWED_DIFFERENCES:
            errors.append(f"{key}: invalid difference classification")
            continue
        if not rationale:
            errors.append(f"{key}: classified difference needs a rationale")
            continue
        classified.append(
            {
                "decision": key,
                "classification": classification,
                "fields": differences,
                "rationale": rationale,
            })
        if classification == "regression_requiring_correction":
            errors.append(f"{key}: regression requires correction")

    counts: dict[str, int] = {}
    for item in classified:
        name = item["classification"]
        counts[name] = counts.get(name, 0) + 1
    return {"classifications": classified, "counts": counts, "errors": errors}


def main() -> int:
    """Summary:
        Performs the main operation for this subsystem.

    Args:
        None.

    Returns:
        int

    Raises:
        None documented; dependency failures may propagate.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("current", type=Path)
    parser.add_argument("annotations", type=Path)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()

    report = compare(
        json.loads(arguments.baseline.read_text(encoding="utf-8")),
        json.loads(arguments.current.read_text(encoding="utf-8")),
        json.loads(arguments.annotations.read_text(encoding="utf-8")),
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        arguments.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
