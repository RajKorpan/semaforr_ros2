#!/usr/bin/env python3
"""Fast source-quality checks for refactored SemaFORR production code."""

import argparse
import re
from pathlib import Path


FORBIDDEN = {
    re.compile(r"\bstd::(?:cout|cerr|clog)\b"): "direct console output",
    re.compile(r"\busing namespace\b"): "namespace import in production code",
    re.compile(r"(?<![\w:])new\s+"): "raw new expression",
    re.compile(r"(?<![\w:])delete\s+"): "raw delete expression",
}


def manifest_paths(source: Path, manifest: Path) -> list[Path]:
    paths = []
    for line in manifest.read_text(encoding="utf-8").splitlines():
        item = line.strip()
        if item and not item.startswith("#"):
            paths.append(source / item)
    return paths


def check(paths: list[Path]) -> list[str]:
    errors = []
    for path in paths:
        if not path.is_file():
            errors.append(f"{path}: manifest entry does not exist")
            continue
        for number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1):
            if line.rstrip() != line:
                errors.append(f"{path}:{number}: trailing whitespace")
            if "\t" in line:
                errors.append(f"{path}:{number}: tab character")
            for pattern, description in FORBIDDEN.items():
                if pattern.search(line):
                    errors.append(f"{path}:{number}: {description}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path(__file__).resolve().parents[1] /
        "config" / "quality_gate_manifest.txt")
    arguments = parser.parse_args()
    errors = check(manifest_paths(arguments.source, arguments.manifest))
    if errors:
        print("\n".join(errors))
        return 1
    print(f"source quality: {len(manifest_paths(arguments.source, arguments.manifest))} files passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
