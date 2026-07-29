#!/usr/bin/env python3
"""Convert the five legacy SemaFORR configuration files to ROS parameter YAML."""

import argparse
import json
from pathlib import Path


def rows(path):
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            yield line.split()


def parse_parameters(path):
    parsed = {}
    for fields in rows(path):
        key, *values = fields
        parsed[key] = values
    return parsed


def scalar(parameters, key, cast=float):
    values = parameters.get(key)
    if values is None or len(values) != 1:
        raise ValueError(f"{key}: expected exactly one value")
    return cast(values[0])


def flag(parameters, key):
    value = scalar(parameters, key, int)
    if value not in (0, 1):
        raise ValueError(f"{key}: expected 0 or 1")
    return bool(value)


def emit(name, value, indentation=6):
    encoded = json.dumps(value)
    return f"{' ' * indentation}{name}: {encoded}"


def convert(arguments):
    parameters = parse_parameters(arguments.parameters)
    dimensions_rows = list(rows(arguments.dimensions))
    if len(dimensions_rows) != 1 or len(dimensions_rows[0]) != 3:
        raise ValueError("dimensions: expected one row with length height granularity")
    length, height, granularity = dimensions_rows[0]

    advisors = []
    for fields in rows(arguments.advisors):
        if len(fields) != 8:
            raise ValueError("advisor row: expected eight fields")
        advisors.append(
            {
                "name": fields[0],
                "enabled": fields[2] == "t",
                "weight": float(fields[3]),
                "parameters": [float(value) for value in fields[4:]],
            }
        )

    move = [float(value) for value in parameters["move"]]
    rotate = [float(value) for value in parameters["rotate"]]
    if move and move[0] == 0.0:
        move.pop(0)
    if rotate and rotate[0] == 0.0:
        rotate.pop(0)

    planner_names = {
        "CUSUM": "cusum",
        "hallwayskel": "hallway_skeleton",
    }
    planner_keys = (
        "distance", "smooth", "novel", "density", "risk", "flow",
        "combined", "CUSUM", "discount", "explore", "spatial",
        "hallwayer", "trailer", "barrier", "conveys", "safe",
        "skeleton", "hallwayskel",
    )
    enabled_planners = [
        planner_names.get(key, key)
        for key in planner_keys
        if flag(parameters, key)
    ]

    feature_keys = {
        "trails": "trailsOn",
        "conveyors": "conveyorsOn",
        "regions": "regionsOn",
        "doors": "doorsOn",
        "hallways": "hallwaysOn",
        "barriers": "barrsOn",
        "astar": "aStarOn",
        "highways": "highwaysOn",
        "frontiers": "frontiersOn",
        "out_of_here": "outofhereOn",
        "doorway": "doorwayOn",
        "find_a_way": "findawayOn",
        "behind": "behindOn",
        "dont_go_back": "dontgobackOn",
    }

    lines = [
        "semaforr:",
        "  ros__parameters:",
        "    configuration:",
        "      use_legacy_files: false",
        "    map:",
        emit("path", str(arguments.map), 6),
        emit("length_m", int(length), 6),
        emit("height_m", int(height), 6),
        emit("granularity_m", float(granularity), 6),
        "    mission:",
        emit("tasks_path", str(arguments.tasks), 6),
        emit("decision_limit", scalar(parameters, "decisionlimit", int), 6),
        emit("plan_limit", scalar(parameters, "planLimit", int), 6),
        "    actions:",
        emit("move_distances_m", move, 6),
        emit("rotation_angles_rad", rotate, 6),
        "    safety:",
        emit("visibility_epsilon_m", scalar(parameters, "canSeePointEpsilon"), 6),
        emit("laser_angle_increment_rad", scalar(parameters, "laserScanRadianIncrement"), 6),
        emit("robot_radius_m", scalar(parameters, "robotFootPrint"), 6),
        emit("obstacle_buffer_m", scalar(parameters, "bufferForRobot"), 6),
        emit("max_laser_range_m", scalar(parameters, "maxLaserRange"), 6),
        emit("max_forward_buffer_m", scalar(parameters, "maxForwardActionBuffer"), 6),
        emit("max_forward_sweep_rad", scalar(parameters, "maxForwardActionSweepAngle"), 6),
        "    learning:",
        emit("highway_distance_threshold_m", scalar(parameters, "highwayDistanceThreshold"), 6),
        emit("highway_time_threshold_s", scalar(parameters, "highwayTimeThreshold"), 6),
        emit("highway_decision_threshold", scalar(parameters, "highwayDecisionThreshold"), 6),
        "    features:",
    ]
    lines.extend(
        emit(name, flag(parameters, legacy), 6)
        for name, legacy in feature_keys.items()
    )
    if enabled_planners:
        lines.extend(
            ["    planners:", emit("enabled", enabled_planners, 6)]
        )
    lines.extend(
        [
            "    advisors:",
            emit("names", [advisor["name"] for advisor in advisors], 6),
            emit("enabled", [advisor["enabled"] for advisor in advisors], 6),
            emit("weights", [advisor["weight"] for advisor in advisors], 6),
            emit(
                "parameters",
                [
                    value
                    for advisor in advisors
                    for value in advisor["parameters"]
                ],
                6,
            ),
        ]
    )
    arguments.output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--advisors", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--dimensions", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--tasks", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    convert(parser.parse_args())


if __name__ == "__main__":
    main()
