"""Compatibility notice for the retired standalone crowd-field adapter."""

import rclpy
from rclpy.node import Node


class CrowdPackageCompatibilityNode(Node):
    """Explain that crowd learning and diagnostics are owned by SemaFORR."""

    def __init__(self):
        super().__init__('semaforr_crowd_compatibility')
        self.get_logger().warning(
            'The standalone crowd adapter is retired. SemaFORR owns crowd '
            'observations, learning, planning models, and diagnostics.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CrowdPackageCompatibilityNode()
    try:
        rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
