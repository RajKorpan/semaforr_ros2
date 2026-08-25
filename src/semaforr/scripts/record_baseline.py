#!/usr/bin/env python3

"""SemaFORR module overview.

Summary:
    This file implements record baseline behavior for developer tooling and experiment automation. It centers on `_load_environment_walls`, `_segment_intersection_distance`, `BaselineRecorder`, `parse_arguments`, `main`, `__init__`, `_elapsed`, `_command_callback`. Its package-relative location is `scripts/record_baseline.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""










import argparse
import json
import math
from pathlib import Path
import statistics
import time
import xml.etree.ElementTree as ET


def _load_environment_walls(path):
    """Summary:
        Performs the load environment walls operation for this subsystem.

    Args:
        path (Any): Supplies path input to the operation.

    Returns:
        Any

    Raises:
        ValueError: If required input or state is invalid.
    """
    if path is None:
        return []
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ValueError(f"cannot load simulator environment map '{path}': {error}") from error
    walls = []
    for obstacle in root.iter("Obstacle"):
        vertices = []
        for vertex in obstacle.findall("Vertex"):
            try:
                point = (float(vertex.attrib["p_x"]), float(vertex.attrib["p_y"]))
            except (KeyError, ValueError) as error:
                raise ValueError(
                    f"malformed simulator obstacle in '{path}'"
                ) from error
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"non-finite simulator obstacle in '{path}'")
            vertices.append(point)
        walls.extend(zip(vertices, vertices[1:]))
        if obstacle.attrib.get("closed") == "1" and len(vertices) > 2:
            walls.append((vertices[-1], vertices[0]))
    if not walls:
        raise ValueError(f"simulator environment map '{path}' has no walls")
    return walls


def _segment_intersection_distance(origin, heading, maximum, wall):
    """Summary:
        Performs the segment intersection distance operation for this subsystem.

    Args:
        origin (Any): Supplies origin input to the operation.
        heading (Any): Supplies heading input to the operation.
        maximum (Any): Supplies maximum input to the operation.
        wall (Any): Supplies wall input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    direction = (math.cos(heading), math.sin(heading))
    edge = (wall[1][0] - wall[0][0], wall[1][1] - wall[0][1])
    denominator = direction[0] * edge[1] - direction[1] * edge[0]
    if abs(denominator) < 1.0e-12:
        return None
    offset = (wall[0][0] - origin[0], wall[0][1] - origin[1])
    ray_distance = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator
    wall_fraction = (
        offset[0] * direction[1] - offset[1] * direction[0]
    ) / denominator
    if 0.0 <= ray_distance <= maximum and 0.0 <= wall_fraction <= 1.0:
        return ray_distance
    return None

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from semaforr_msgs.msg import DecisionRecord, NavigationState


class BaselineRecorder(Node):
    """Summary:
        Encapsulates baseline recorder state and behavior for this subsystem.

    Args:
        None.

    Returns:
        Not applicable; classes construct instances.

    Raises:
        None documented; dependency failures may propagate.
    """
    TICK_HZ = 20.0
    TICK_PERIOD_S = 1.0 / TICK_HZ

    def __init__(
        self,
        output_path,
        duration,
        sensor_cutoff=None,
        initial_x=100.0,
        initial_y=100.0,
        initial_yaw=0.0,
        scenario_name="stage_tutorial_open_space",
        environment_map=None,
    ):
        """Summary:
            Performs the init operation for this subsystem.

        Args:
            output_path (Any): Supplies output path input to the operation.
            duration (Any): Supplies duration input to the operation.
            sensor_cutoff (Any): Supplies sensor cutoff input to the operation.
            initial_x (Any): Supplies initial x input to the operation.
            initial_y (Any): Supplies initial y input to the operation.
            initial_yaw (Any): Supplies initial yaw input to the operation.
            scenario_name (Any): Supplies scenario name input to the operation.
            environment_map (Any): Supplies environment map input to the operation.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        super().__init__("semaforr_baseline_recorder")
        self._output_path = output_path
        self._duration = duration
        self._sensor_cutoff = sensor_cutoff
        self._wall_started_at = time.monotonic()
        self._tick_count = 0
        self._initial_pose = [initial_x, initial_y, initial_yaw]
        self._scenario_name = scenario_name
        self._environment_map = environment_map
        self._environment_walls = _load_environment_walls(environment_map)
        self._pose = list(self._initial_pose)
        self._command = Twist()
        self._last_command_signature = None
        self._commands = []
        self._decisions = []
        self._pose_messages = []
        self._computation_times = []
        self._navigation_states = []
        self._finished = False
        self._trace_written = False

        self._pose_publisher = self.create_publisher(PoseStamped, "/pose", 10)
        self._scan_publisher = self.create_publisher(
            LaserScan, "/scan_raw", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._command_callback, 10)
        self.create_subscription(
            DecisionRecord,
            "/decision_records",
            self._decision_callback,
            10,
        )
        self.create_subscription(
            NavigationState,
            "/navigation_state",
            self._navigation_state_callback,
            10,
        )
        self.create_timer(self.TICK_PERIOD_S, self._tick)

    def _elapsed(self):
        """Summary:
            Performs the elapsed operation for this subsystem.

        Args:
            None.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        return self._tick_count * self.TICK_PERIOD_S

    def _command_callback(self, message):
        """Summary:
            Performs the command callback operation for this subsystem.

        Args:
            message (Any): Supplies message input to the operation.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        self._command = message
        signature = (
            round(message.linear.x, 6),
            round(message.linear.y, 6),
            round(message.angular.z, 6),
        )
        if signature == self._last_command_signature:
            return
        self._last_command_signature = signature
        self._commands.append(
            {
                "time_s": round(self._elapsed(), 3),
                "linear_x": signature[0],
                "linear_y": signature[1],
                "angular_z": signature[2],
            }
        )

    def _decision_callback(self, message):
        """Summary:
            Performs the decision callback operation for this subsystem.

        Args:
            message (Any): Supplies message input to the operation.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        tier_names = {
            DecisionRecord.TIER_ONE: "tier_one",
            DecisionRecord.TIER_TWO: "tier_two",
            DecisionRecord.TIER_THREE: "tier_three",
            DecisionRecord.EXPLORATION: "exploration",
            DecisionRecord.FALLBACK: "fallback",
            DecisionRecord.SAFE_STOP: "safe_stop",
        }
        source_names = {
            DecisionRecord.SOURCE_MANDATORY_RULE: "mandatory_rule",
            DecisionRecord.SOURCE_TIER_THREE_ADVISOR:
                "tier_three_advisor",
            DecisionRecord.SOURCE_PLANNER: "planner",
            DecisionRecord.SOURCE_EXPLORATION: "exploration",
            DecisionRecord.SOURCE_FALLBACK: "fallback",
            DecisionRecord.SOURCE_SAFE_STOP: "safe_stop",
        }
        outcome_names = {
            DecisionRecord.OUTCOME_PENDING: "pending",
            DecisionRecord.OUTCOME_COMPLETED: "completed",
            DecisionRecord.OUTCOME_TIMED_OUT: "timed_out",
            DecisionRecord.OUTCOME_ODOMETRY_RESET: "odometry_reset",
            DecisionRecord.OUTCOME_CLOCK_RESET: "clock_reset",
            DecisionRecord.OUTCOME_CANCELLED: "cancelled",
            DecisionRecord.OUTCOME_SENSOR_LOST: "sensor_lost",
            DecisionRecord.OUTCOME_SHUTDOWN: "shutdown",
        }
        computation_time = message.decision_latency_s
        self._computation_times.append(computation_time)
        task = message.task if message.has_task else None
        self._decisions.append(
            {
                "time_s": round(self._elapsed(), 3),
                "sequence": message.sequence,
                "task": task.task_index if task else None,
                "decision": task.decision_count if task else None,
                "computation_time_s": computation_time,
                "planning_latency_s": message.planning_latency_s,
                "model_update_cost_s": message.model_update_cost_s,
                "allocation_count": message.allocation_count,
                "allocation_bytes": message.allocation_bytes,
                "covered_cells": message.covered_cells,
                "phase_events": list(message.phase_events),
                "target": (
                    [task.target.x, task.target.y] if task else None
                ),
                "waypoint": (
                    [task.waypoint.x, task.waypoint.y]
                    if task and task.has_waypoint
                    else None
                ),
                "robot_pose": [
                    message.robot_pose.x,
                    message.robot_pose.y,
                    message.robot_pose.theta,
                ],
                "candidate_actions": [
                    [action.type, action.magnitude_index]
                    for action in message.candidates
                ],
                "decision_tier": tier_names[message.selected_tier],
                "decision_source": source_names[message.selected_source],
                "selected_policy": message.selected_policy,
                "vetoes": [
                    {
                        "action": [
                            veto.action.type,
                            veto.action.magnitude_index,
                        ],
                        "rule": veto.rule,
                        "explanation": veto.explanation,
                    }
                    for veto in message.vetoes
                ],
                "chosen_action": [
                    message.selected_action.type,
                    message.selected_action.magnitude_index,
                ],
                "advisor_contributions": [
                    {
                        "advisor": contribution.advisor,
                        "action": [
                            contribution.action.type,
                            contribution.action.magnitude_index,
                        ],
                        "raw_score": contribution.raw_score,
                        "weight": contribution.weight,
                        "weighted_score": contribution.weighted_score,
                        "explanation": contribution.explanation,
                    }
                    for contribution in message.advisor_contributions
                ],
                "chosen_planner": (
                    message.selected_planner if message.has_planner else None
                ),
                "action_outcome": outcome_names[message.action_outcome],
                "action_duration_s": message.action_duration_s,
                "action_progress": message.action_progress,
                "action_target": message.action_target,
                "outcome_detail": message.outcome_detail,
            }
        )

    def _navigation_state_callback(self, message):
        """Summary:
            Performs the navigation state callback operation for this subsystem.

        Args:
            message (Any): Supplies message input to the operation.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        state_names = {
            NavigationState.WAITING_FOR_SENSORS: "waiting_for_sensors",
            NavigationState.READY_TO_DECIDE: "ready_to_decide",
            NavigationState.EXECUTING_ACTION: "executing_action",
            NavigationState.STOPPED: "stopped",
        }
        state = state_names[message.state]
        if (
            self._navigation_states
            and self._navigation_states[-1]["state"] == state
            and self._navigation_states[-1]["detail"] == message.detail
        ):
            return
        self._navigation_states.append(
            {
                "time_s": round(self._elapsed(), 3),
                "sequence": message.transition_sequence,
                "state": state,
                "detail": message.detail,
                "failure": message.failure,
            }
        )

    def _tick(self):
        """Summary:
            Performs the tick operation for this subsystem.

        Args:
            None.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        linear_x = self._command.linear.x
        angular_z = self._command.angular.z
        self._pose[2] = math.atan2(
            math.sin(self._pose[2] + angular_z * self.TICK_PERIOD_S),
            math.cos(self._pose[2] + angular_z * self.TICK_PERIOD_S),
        )
        proposed_x = self._pose[0] + (
            linear_x * math.cos(self._pose[2]) * self.TICK_PERIOD_S
        )
        proposed_y = self._pose[1] + (
            linear_x * math.sin(self._pose[2]) * self.TICK_PERIOD_S
        )
        motion_distance = math.hypot(
            proposed_x - self._pose[0], proposed_y - self._pose[1]
        )
        blocked = any(
            _segment_intersection_distance(
                self._pose[:2], self._pose[2], motion_distance, wall
            ) is not None
            for wall in self._environment_walls
        )
        if not blocked:
            self._pose[0] = proposed_x
            self._pose[1] = proposed_y
        self._tick_count += 1

        sensors_enabled = (
            self._sensor_cutoff is None
            or self._elapsed() <= self._sensor_cutoff
        )
        if not sensors_enabled:
            if self._elapsed() >= self._duration:
                self._finished = True
            return

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = self._pose[0]
        pose.pose.position.y = self._pose[1]
        pose.pose.orientation.z = math.sin(self._pose[2] / 2.0)
        pose.pose.orientation.w = math.cos(self._pose[2] / 2.0)
        self._pose_publisher.publish(pose)
        self._pose_messages.append(
            {
                "time_s": round(self._elapsed(), 3),
                "pose": [round(value, 9) for value in self._pose],
            }
        )

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_laser_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 540.0
        scan.range_min = 0.05
        scan.range_max = 5.0
        scan.ranges = []
        for index in range(1081):
            heading = self._pose[2] + scan.angle_min + index * scan.angle_increment
            intersections = [
                distance
                for wall in self._environment_walls
                if (distance := _segment_intersection_distance(
                    self._pose[:2], heading, scan.range_max, wall
                )) is not None
            ]
            scan.ranges.append(min(intersections, default=scan.range_max))
        self._scan_publisher.publish(scan)

        if self._elapsed() >= self._duration:
            self._finished = True

    @property
    def finished(self):
        """Summary:
            Performs the finished operation for this subsystem.

        Args:
            None.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        return self._finished

    def write_trace(self):
        """Summary:
            Writes trace for this subsystem.

        Args:
            None.

        Returns:
            Any

        Raises:
            None documented; dependency failures may propagate.
        """
        if self._trace_written:
            return
        self._trace_written = True

        metrics = {
            "decision_count": len(self._decisions),
            "simulated_duration_s": round(self._elapsed(), 6),
            "wall_duration_s": round(
                time.monotonic() - self._wall_started_at, 6
            ),
        }
        if self._computation_times:
            metrics["decision_computation_s"] = {
                "minimum": min(self._computation_times),
                "median": statistics.median(self._computation_times),
                "mean": statistics.fmean(self._computation_times),
                "maximum": max(self._computation_times),
            }
        metrics["planning_latency_s"] = sum(
            decision["planning_latency_s"] for decision in self._decisions
        )
        metrics["model_update_cost_s"] = sum(
            decision["model_update_cost_s"] for decision in self._decisions
        )
        metrics["allocations"] = {
            "count": sum(
                decision["allocation_count"] for decision in self._decisions
            ),
            "bytes": sum(
                decision["allocation_bytes"] for decision in self._decisions
            ),
        }
        metrics["coverage_cells"] = max(
            (decision["covered_cells"] for decision in self._decisions),
            default=0,
        )
        attempted_tasks = {
            decision["task"]
            for decision in self._decisions
            if decision["task"] is not None
        }
        completed_targets = sum(
            event == "target_completed"
            for decision in self._decisions
            for event in decision["phase_events"]
        )
        metrics["targets"] = {
            "attempted": len(attempted_tasks),
            "succeeded": completed_targets,
            "success_rate": (
                completed_targets / len(attempted_tasks)
                if attempted_tasks else 0.0
            ),
        }
        metrics["distance_m"] = sum(
            math.hypot(
                current["pose"][0] - previous["pose"][0],
                current["pose"][1] - previous["pose"][1],
            )
            for previous, current in zip(
                self._pose_messages, self._pose_messages[1:]
            )
        )
        interventions = {}
        for decision in self._decisions:
            name = decision["selected_policy"]
            interventions[name] = interventions.get(name, 0) + 1
        metrics["intervention_frequency"] = {
            name: count / len(self._decisions)
            for name, count in sorted(interventions.items())
        } if self._decisions else {}

        task_transitions = []
        previous_task = None
        for decision in self._decisions:
            if decision["task"] != previous_task:
                task_transitions.append(
                    {
                        "task": decision["task"],
                        "first_decision": decision["decision"],
                    }
                )
                previous_task = decision["task"]

        trace = {
            "schema_version": 4,
            "scenario": {
                "name": self._scenario_name,
                "duration_s": self._duration,
                "tick_hz": self.TICK_HZ,
                "initial_pose": self._initial_pose,
                "laser_range_m": 5.0,
                "laser_sample_count": 1081,
                "sensor_cutoff_s": self._sensor_cutoff,
                "simulator_environment_map": (
                    str(self._environment_map) if self._environment_map else None
                ),
                "sensor_fixture": {
                    "pose_topic": "/pose",
                    "laser_topic": "/scan_raw",
                    "pose_integration": "fixed-step differential drive",
                    "laser_angle_min_rad": -math.pi,
                    "laser_angle_max_rad": math.pi,
                    "laser_angle_increment_rad": math.pi / 540.0,
                    "laser_range_min_m": 0.05,
                    "laser_ranges": {"repeat": 5.0, "count": 1081},
                },
            },
            "result": {
                "final_pose": [round(value, 6) for value in self._pose],
                "input_messages": {
                    "poses": self._pose_messages,
                    "laser": {
                        "published_at_pose_times": True,
                        "range_m": 5.0,
                        "sample_count": 1081,
                    },
                },
                "commands": self._commands,
                "decisions": self._decisions,
                "task_transitions": task_transitions,
                "navigation_states": self._navigation_states,
                "metrics": metrics,
            },
        }

        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._output_path.write_text(
            json.dumps(trace, indent=2) + "\n", encoding="utf-8"
        )
        self.get_logger().info(
            f"Wrote SemaFORR baseline trace to {self._output_path}"
        )


def parse_arguments(arguments):
    """Summary:
        Parses arguments for this subsystem.

    Args:
        arguments (Any): Supplies arguments input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--initial-x", type=float, default=100.0)
    parser.add_argument("--initial-y", type=float, default=100.0)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument(
        "--scenario-name",
        default="stage_tutorial_open_space",
    )
    parser.add_argument(
        "--environment-map",
        type=Path,
        default=None,
        help="Simulator-only XML geometry; this is never sent to SemaFORR",
    )
    parser.add_argument(
        "--sensor-cutoff",
        type=float,
        default=-1.0,
        help=(
            "Stop publishing pose and scan messages after this time; "
            "negative disables the cutoff"
        ),
    )
    parsed = parser.parse_args(arguments)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")
    if parsed.sensor_cutoff >= 0.0:
        if parsed.sensor_cutoff == 0.0:
            parser.error("--sensor-cutoff must be positive or negative")
        if parsed.sensor_cutoff >= parsed.duration:
            parser.error("--sensor-cutoff must be less than --duration")
    else:
        parsed.sensor_cutoff = None
    return parsed


def main(args=None):
    """Summary:
        Performs the main operation for this subsystem.

    Args:
        args (Any): Supplies args input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    rclpy.init(args=args)
    parsed = parse_arguments(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = BaselineRecorder(
        parsed.output,
        parsed.duration,
        parsed.sensor_cutoff,
        parsed.initial_x,
        parsed.initial_y,
        parsed.initial_yaw,
        parsed.scenario_name,
        parsed.environment_map,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_trace()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
