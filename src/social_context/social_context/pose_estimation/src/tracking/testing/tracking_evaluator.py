# FOR TESTING AND EVALUATION PURPOSES ONLY
#!/usr/bin/env python3
"""
Tracking Evaluator Node

Subscribes to HuNav's ground-truth agent positions (/human_states) and
your tracker's output (/human_poses_3d_tracked_global), matches them by
nearest position each frame, and measures:
  - ID switch count (how often a ground-truth person's closest tracker
    ID changes between frames)
  - Position error (distance between ground-truth and matched tracker
    position)
"""

import rclpy
from rclpy.node import Node
from social_context_msgs.msg import TrackedPersonArray
from hunav_msgs.msg import Agents
import csv
import time
import numpy as np
from scipy.optimize import linear_sum_assignment

class TrackingEvaluator(Node):
    def __init__(self):
        super().__init__('tracking_evaluator')

        self.latest_ground_truth = {}    # {gt_id: (x, y)}
        self.gt_to_tracker_id = {}       # {gt_id : tracker_id}
        self.id_switch_count = 0
        self.position_errors = []
        self.records = []

        self.max_match_distance = 3.0 

        self.gt_sub = self.create_subscription(
            Agents, '/human_states', self.ground_truth_callback, 10
        )

        self.tracked_sub = self.create_subscription(
            TrackedPersonArray, '/human_poses_3d_tracked_global', self.tracked_callback, 10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Tracking Evaluator Ready')
        self.get_logger().info('  Ground truth: /human_states')
        self.get_logger().info('  Tracker output: /human_poses_3d_tracked_global')
        self.get_logger().info('=' * 60)

    def ground_truth_callback(self, msg: Agents):
        for agent in msg.agents:
            self.latest_ground_truth[agent.id] = (agent.position.position.x, agent.position.position.y)

    def tracked_callback(self, msg: TrackedPersonArray):
        if not self.latest_ground_truth:
            return

        if len(msg.people) == 0:
            return


        gt_ids = list(self.latest_ground_truth.keys())
        gt_positions = [self.latest_ground_truth[gid] for gid in gt_ids]
        tracker_ids = [person.id for person in msg.people]
        tracker_positions = [(person.x, person.y) for person in msg.people]
 
        # Build cost matrix: rows = ground-truth people, columns = tracked people
        cost_matrix = np.zeros((len(gt_positions), len(tracker_positions)))
        for i, gt_pos in enumerate(gt_positions):
            for j, tr_pos in enumerate(tracker_positions):
                cost_matrix[i, j] = np.linalg.norm(np.array(gt_pos) - np.array(tr_pos))
 
        # Exclusive one-to-one assignment - no two ground-truth people can
        # claim the same tracker, and no tracker gets claimed twice
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
 
        for i, j in zip(row_indices, col_indices):
            dist = cost_matrix[i, j]
            if dist > self.max_match_distance:
                continue  # no reasonable match for this gt person this frame
 
            gt_id = gt_ids[i]
            best_tracker_id = tracker_ids[j]
 
            self.position_errors.append(dist)

            # ID switch detection
            if gt_id in self.gt_to_tracker_id:
                previous_tracker_id = self.gt_to_tracker_id[gt_id]
                if previous_tracker_id != best_tracker_id:
                    self.id_switch_count += 1
                    self.get_logger().warn(
                        f'ID SWITCH: ground-truth person {gt_id} was tracker '
                        f'{previous_tracker_id}, now tracker {best_tracker_id} '
                        f'(dist={dist:.3f}m)'
                    )

            self.gt_to_tracker_id[gt_id] = best_tracker_id

            self.records.append({
                'time': time.time(),
                'gt_id': gt_id,
                'tracker_id': best_tracker_id,
                'position_error': dist
            })


    def save_results(self):
        output_path = '/tmp/tracking_eval_results.csv'
        try:
            with open(output_path, 'w', newline='') as f:
                writer = csv.DictWriter(
                    f, fieldnames=['time', 'gt_id', 'tracker_id', 'position_error']
                )
                writer.writeheader()
                writer.writerows(self.records)
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {e}')

        avg_error = (
            sum(self.position_errors) / len(self.position_errors)
            if self.position_errors else 0.0
        )
        max_error = max(self.position_errors) if self.position_errors else 0.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('EVALUATION SUMMARY')
        self.get_logger().info(f'  Total frames logged: {len(self.records)}')
        self.get_logger().info(f'  ID switches: {self.id_switch_count}')
        self.get_logger().info(f'  Average position error: {avg_error:.3f}m')
        self.get_logger().info(f'  Max position error: {max_error:.3f}m')
        self.get_logger().info(f'  Results saved to: {output_path}')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()