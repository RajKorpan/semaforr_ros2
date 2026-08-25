#!/usr/bin/env python3
"""
F-Formation detection node.

Subscribes to: /human_poses_3d_tracked_global (TrackedPersonArray)
Publishes to: /formation_groups (FormationGroupArray)
"""

import math
from collections import defaultdict, deque

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

import rclpy
from rclpy.node import Node

from social_context_msgs.msg import TrackedPersonArray, FormationGroup, FormationGroupArray

from social_context.pose_estimation.src.formation.formation_classifier import (
    Person,
    classify_cluster,
)


class FormationDetectorNode(Node):
    def __init__(self):
        super().__init__('formation_detector')

        self.declare_parameter('input_topic', '/human_poses_3d_tracked_global')
        self.declare_parameter('output_topic', '/formation_groups')
        self.declare_parameter('cluster_eps', 1.5)          # meters
        self.declare_parameter('min_cluster_size', 2)
        self.declare_parameter('distance_threshold', 1.5)   # meters, formation-level distance gate
        self.declare_parameter('angle_threshold_deg', 60.0)
        self.declare_parameter('stability_window', 5)        # classifications per group before reporting

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.cluster_eps = self.get_parameter('cluster_eps').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold_deg = self.get_parameter('angle_threshold_deg').value
        self.stability_window = self.get_parameter('stability_window').value

        self._history = defaultdict(lambda: deque(maxlen=self.stability_window))

        self.sub = self.create_subscription(TrackedPersonArray, input_topic, self.callback, 10)
        self.pub = self.create_publisher(FormationGroupArray, output_topic, 10)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Formation Detector Node Ready!')
        self.get_logger().info(f'  Subscribing to: {input_topic}')
        self.get_logger().info(f'  Publishing to: {output_topic}')
        self.get_logger().info(f'  cluster_eps={self.cluster_eps}m, min_cluster_size={self.min_cluster_size}')
        self.get_logger().info(f'  stability_window={self.stability_window}')
        self.get_logger().info('=' * 60)

    def callback(self, msg: TrackedPersonArray):
        people = [
            Person(
                id=p.id,
                x=p.x,
                y=p.y,
                yaw=2.0 * math.atan2(p.orientation_z, p.orientation_w),
            )
            for p in msg.people
        ]

        groups_out = FormationGroupArray()
        groups_out.header = msg.header

        if len(people) < 2:
            self.pub.publish(groups_out)
            return

        positions = np.array([[p.x, p.y] for p in people])
        labels = self._cluster(positions)

        current_group_keys = set()
        for label in sorted(set(labels)):
            if label == -1:
                continue  # not part of any cluster
            member_indices = [i for i, l in enumerate(labels) if l == label]
            if len(member_indices) < self.min_cluster_size:
                continue
            cluster_people = [people[i] for i in member_indices]

            group_key = frozenset(p.id for p in cluster_people)
            current_group_keys.add(group_key)

            raw_type = classify_cluster(cluster_people, self.distance_threshold, self.angle_threshold_deg)
            self._history[group_key].append(raw_type)
            stable_type, confidence = self._resolve_classification(self._history[group_key])

            group_msg = FormationGroup()
            group_msg.member_ids = [p.id for p in cluster_people]
            group_msg.formation_type = stable_type.value
            group_msg.confidence = confidence
            center = positions[member_indices].mean(axis=0)
            group_msg.center_x = float(center[0])
            group_msg.center_y = float(center[1])
            groups_out.groups.append(group_msg)

        # drop history for groups that no longer exist
        for group_key in [k for k in self._history if k not in current_group_keys]:
            del self._history[group_key]

        self.pub.publish(groups_out)

    def _cluster(self, positions: np.ndarray) -> np.ndarray:
        """Connected-components clustering: two people are linked if within
        cluster_eps of each other, clusters are the transitive closure of
        that link (same practical grouping behavior as DBSCAN for this use
        case). Returns a label per row of positions."""
        n = len(positions)
        dists = np.linalg.norm(positions[:, None] - positions[None, :], axis=2)
        adjacency = (dists <= self.cluster_eps) & (dists > 0)
        graph = csr_matrix(adjacency)
        n_components, labels = connected_components(graph, directed=False)
        return labels

    @staticmethod
    def _resolve_classification(history):
        """Majority formation type over recent history, with confidence =
        fraction of the window agreeing with it."""
        counts = defaultdict(int)
        for t in history:
            counts[t] += 1
        best_type, best_count = max(counts.items(), key=lambda kv: kv[1])
        confidence = best_count / len(history)
        return best_type, confidence


def main(args=None):
    rclpy.init(args=args)
    node = FormationDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
