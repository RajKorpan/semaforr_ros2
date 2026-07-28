#!/usr/bin/env python3

"""Drive a deterministic virtual robot and record SemaFORR's observable output."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class BaselineRecorder(Node):
    def __init__(self, output_path, duration):
        super().__init__("semaforr_baseline_recorder")
        self._output_path = output_path
        self._duration = duration
        self._started_at = time.monotonic()
        self._last_tick = self._started_at
        self._pose = [100.0, 100.0, 0.0]
        self._command = Twist()
        self._last_command_signature = None
        self._commands = []
        self._decisions = []
        self._computation_times = []
        self._finished = False
        self._trace_written = False

        self._pose_publisher = self.create_publisher(PoseStamped, "/pose", 10)
        self._scan_publisher = self.create_publisher(
            LaserScan, "/scan_raw", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._command_callback, 10)
        self.create_subscription(
            String, "/decision_log", self._decision_callback, 10
        )
        self.create_timer(0.05, self._tick)

    def _elapsed(self):
        return time.monotonic() - self._started_at

    def _command_callback(self, message):
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
        fields = message.data.split("\t")
        if len(fields) < 16:
            self.get_logger().warning(
                "Ignoring malformed decision_log message with "
                f"{len(fields)} fields"
            )
            return

        computation_time = float(fields[3])
        self._computation_times.append(computation_time)
        self._decisions.append(
            {
                "time_s": round(self._elapsed(), 3),
                "task": int(fields[0]),
                "decision": int(fields[1]),
                "overall_time_s": float(fields[2]),
                "computation_time_s": computation_time,
                "target": [float(fields[4]), float(fields[5])],
                "robot_pose": [
                    float(fields[6]),
                    float(fields[7]),
                    float(fields[8]),
                ],
                "max_forward_parameter": int(fields[9]),
                "decision_tier": float(fields[10]),
                "vetoed_actions": fields[11],
                "chosen_action": [int(fields[12]), int(fields[13])],
                "advisors": fields[14],
                "advisor_comments": fields[15],
                "chosen_planner": fields[25] if len(fields) > 25 else "",
                "planner_comments": fields[28] if len(fields) > 28 else "",
            }
        )

    def _tick(self):
        now = time.monotonic()
        delta = min(now - self._last_tick, 0.2)
        self._last_tick = now

        linear_x = self._command.linear.x
        angular_z = self._command.angular.z
        self._pose[2] = math.atan2(
            math.sin(self._pose[2] + angular_z * delta),
            math.cos(self._pose[2] + angular_z * delta),
        )
        self._pose[0] += linear_x * math.cos(self._pose[2]) * delta
        self._pose[1] += linear_x * math.sin(self._pose[2]) * delta

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = self._pose[0]
        pose.pose.position.y = self._pose[1]
        pose.pose.orientation.z = math.sin(self._pose[2] / 2.0)
        pose.pose.orientation.w = math.cos(self._pose[2] / 2.0)
        self._pose_publisher.publish(pose)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_laser_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 540.0
        scan.range_min = 0.05
        scan.range_max = 5.0
        scan.ranges = [5.0] * 1081
        self._scan_publisher.publish(scan)

        if self._elapsed() >= self._duration:
            self._finished = True

    @property
    def finished(self):
        return self._finished

    def write_trace(self):
        if self._trace_written:
            return
        self._trace_written = True

        metrics = {
            "decision_count": len(self._decisions),
            "wall_duration_s": round(self._elapsed(), 6),
        }
        if self._computation_times:
            metrics["decision_computation_s"] = {
                "minimum": min(self._computation_times),
                "median": statistics.median(self._computation_times),
                "mean": statistics.fmean(self._computation_times),
                "maximum": max(self._computation_times),
            }

        trace = {
            "schema_version": 1,
            "scenario": {
                "name": "stage_tutorial_open_space",
                "duration_s": self._duration,
                "tick_hz": 20.0,
                "initial_pose": [100.0, 100.0, 0.0],
                "laser_range_m": 5.0,
                "laser_sample_count": 1081,
            },
            "result": {
                "final_pose": [round(value, 6) for value in self._pose],
                "commands": self._commands,
                "decisions": self._decisions,
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=20.0)
    parsed = parser.parse_args(arguments)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")
    return parsed


def main(args=None):
    rclpy.init(args=args)
    parsed = parse_arguments(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = BaselineRecorder(parsed.output, parsed.duration)
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
