#!/usr/bin/env python3

"""SemaFORR module overview.

Summary:
    This file implements run experiment matrix behavior for developer tooling and experiment automation. It centers on `parse_arguments`, `main`. Its package-relative location is `scripts/run_experiment_matrix.py`.

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
import subprocess


DEFAULT_PROFILES = [
    "purely_reactive",
    "original",
    "doors",
    "least_angle",
    "access",
    "tentative",
    "hallways",
    "shortest_path",
    "cost_graph",
    "wander",
    "deliberator",
    "forward_only",
    "global_exploration",
    "local_exploration",
    "highway",
    "circumstances",
    "naive",
]


def parse_arguments():
    """Summary:
        Parses arguments for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--profiles", nargs="+", default=DEFAULT_PROFILES)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=0)
    parsed = parser.parse_args()
    if parsed.runs <= 0 or parsed.duration <= 0.0 or parsed.seed < 0:
        parser.error("runs and duration must be positive and seed nonnegative")
    return parsed


def main():
    """Summary:
        Performs the main operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        SystemExit: If required input or state is invalid.
    """
    arguments = parse_arguments()
    arguments.output_directory.mkdir(parents=True, exist_ok=True)
    runs = []
    for profile in arguments.profiles:
        for repetition in range(arguments.runs):
            seed = arguments.seed + repetition
            output = arguments.output_directory / (
                f"{profile}-seed-{seed}.json"
            )
            command = [
                "ros2",
                "launch",
                "semaforr",
                "stage_tutorial_baseline.launch.py",
                f"profile:={profile}",
                f"random_seed:={seed}",
                f"duration:={arguments.duration}",
                f"output:={output}",
            ]
            completed = subprocess.run(command, check=False)
            runs.append(
                {
                    "profile": profile,
                    "seed": seed,
                    "output": str(output),
                    "return_code": completed.returncode,
                }
            )
            if completed.returncode != 0:
                raise SystemExit(completed.returncode)
    manifest = {
        "schema_version": 1,
        "duration_s": arguments.duration,
        "runs": runs,
    }
    (arguments.output_directory / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
