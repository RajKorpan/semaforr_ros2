#!/usr/bin/env python3
"""SemaFORR module overview.

Summary:
    This file implements check coverage behavior for developer tooling and experiment automation. It centers on `parse_lcov`, `evaluate`, `main`. Its package-relative location is `scripts/check_coverage.py`.

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


def parse_lcov(path: Path) -> dict[str, tuple[int, int]]:
    """Summary:
        Parses lcov for this subsystem.

    Args:
        path (Path): Supplies path input to the operation.

    Returns:
        dict[str, tuple[int, int]]

    Raises:
        None documented; dependency failures may propagate.
    """
    coverage: dict[str, tuple[int, int]] = {}
    source = None
    found: set[int] = set()
    hit: set[int] = set()
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("SF:"):
            source = line[3:].replace("\\", "/")
            found = set()
            hit = set()
        elif line.startswith("DA:") and source:
            number, count, *_ = line[3:].split(",")
            found.add(int(number))
            if int(count) > 0:
                hit.add(int(number))
        elif line == "end_of_record" and source:
            coverage[source] = (len(hit), len(found))
            source = None
    return coverage


def evaluate(
    coverage: dict[str, tuple[int, int]],
    configuration: dict,
) -> tuple[list[str], dict[str, float]]:
    """Summary:
        Evaluates package content for this subsystem.

    Args:
        coverage (dict[str, tuple[int, int]]): Supplies coverage input to the operation.
        configuration (dict): Supplies configuration input to the operation.

    Returns:
        tuple[list[str], dict[str, float]]

    Raises:
        None documented; dependency failures may propagate.
    """
    errors = []
    percentages = {}
    minimum = float(configuration["minimum_line_percent"])
    for relative in configuration["files"]:
        matches = [
            counts for path, counts in coverage.items()
            if path.endswith("/" + relative) or path == relative
        ]
        if not matches:
            errors.append(f"{relative}: no coverage record")
            continue
        hit = sum(item[0] for item in matches)
        found = sum(item[1] for item in matches)
        percent = 100.0 if found == 0 else hit * 100.0 / found
        percentages[relative] = percent
        if percent + 1.0e-9 < minimum:
            errors.append(
                f"{relative}: {percent:.1f}% is below {minimum:.1f}%")
    return errors, percentages


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
    parser.add_argument("lcov_info", type=Path)
    parser.add_argument(
        "--configuration",
        type=Path,
        default=Path(__file__).resolve().parents[1] /
        "config" / "coverage_thresholds.json")
    arguments = parser.parse_args()
    configuration = json.loads(
        arguments.configuration.read_text(encoding="utf-8"))
    errors, percentages = evaluate(
        parse_lcov(arguments.lcov_info), configuration)
    for path, percent in sorted(percentages.items()):
        print(f"{path}: {percent:.1f}%")
    if errors:
        print("\n".join(errors))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
