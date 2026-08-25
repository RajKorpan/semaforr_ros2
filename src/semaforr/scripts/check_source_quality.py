#!/usr/bin/env python3
"""SemaFORR module overview.

Summary:
    This file implements check source quality behavior for developer tooling and experiment automation. It centers on `manifest_paths`, `check`, `main`. Its package-relative location is `scripts/check_source_quality.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""










import argparse
import re
from pathlib import Path


FORBIDDEN = {
    re.compile(r"\bstd::(?:cout|cerr|clog)\b"): "direct console output",
    re.compile(r"\busing namespace\b"): "namespace import in production code",
    re.compile(
        r"(?<![\w:])new\s+[A-Za-z_:][A-Za-z0-9_:<>]*\s*(?:\(|\[)"
    ): "raw new expression",
    re.compile(r"(?<![\w:])delete\s+[A-Za-z_][A-Za-z0-9_]*\s*;"):
        "raw delete expression",
}


def manifest_paths(source: Path, manifest: Path) -> list[Path]:
    """Summary:
        Performs the manifest paths operation for this subsystem.

    Args:
        source (Path): Supplies source input to the operation.
        manifest (Path): Supplies manifest input to the operation.

    Returns:
        list[Path]

    Raises:
        None documented; dependency failures may propagate.
    """
    paths = []
    for line in manifest.read_text(encoding="utf-8").splitlines():
        item = line.strip()
        if item and not item.startswith("#"):
            candidate = source / item
            if candidate.is_dir():
                paths.extend(
                    path
                    for path in candidate.rglob("*")
                    if path.suffix in {".hpp", ".cpp"}
                )
            else:
                paths.append(candidate)
    return paths


def check(paths: list[Path]) -> list[str]:
    """Summary:
        Performs the check operation for this subsystem.

    Args:
        paths (list[Path]): Supplies paths input to the operation.

    Returns:
        list[str]

    Raises:
        None documented; dependency failures may propagate.
    """
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
