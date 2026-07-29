"""ROS 2 diagnostic adapter for the domain-owned learned crowd field."""

import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from social_context_msgs.msg import CrowdField
from visualization_msgs.msg import Marker, MarkerArray

from semaforr_crowd.model import CrowdFieldSnapshot


class CrowdFieldDiagnosticNode(Node):
    def __init__(self):
        super().__init__('semaforr_crowd')
        self.declare_parameter('field_topic', 'crowd_field')
        self.declare_parameter('density_topic', 'crowd_density')
        self.declare_parameter('risk_topic', 'crowd_risk')
        self.declare_parameter('flow_topic', 'crowd_flow')
        self.declare_parameter('occupancy_scale', 100.0)
        self.declare_parameter('minimum_flow', 1.0e-6)

        self._density_publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('density_topic').value,
            10,
        )
        self._risk_publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('risk_topic').value,
            10,
        )
        self._flow_publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('flow_topic').value,
            10,
        )
        self._subscription = self.create_subscription(
            CrowdField,
            self.get_parameter('field_topic').value,
            self._observe,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def _observe(self, message):
        try:
            snapshot = CrowdFieldSnapshot.from_message(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._density_publisher.publish(
            self._occupancy(message, snapshot, snapshot.density)
        )
        self._risk_publisher.publish(
            self._occupancy(message, snapshot, snapshot.risk)
        )
        self._flow_publisher.publish(
            self._flow_markers(message, snapshot)
        )

    def _occupancy(self, source, snapshot, values):
        result = OccupancyGrid()
        result.header = source.header
        result.info.resolution = snapshot.resolution_m
        result.info.width = snapshot.columns
        result.info.height = snapshot.rows
        result.info.origin.position.x = snapshot.origin_x_m
        result.info.origin.position.y = snapshot.origin_y_m
        result.info.origin.orientation.w = 1.0
        scale = float(self.get_parameter('occupancy_scale').value)
        result.data = [
            int(max(0.0, min(100.0, value * scale)))
            for value in values
        ]
        return result

    def _flow_markers(self, source, snapshot):
        result = MarkerArray()
        clear = Marker()
        clear.header = source.header
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        minimum = float(self.get_parameter('minimum_flow').value)
        for index, (velocity_x, velocity_y) in enumerate(snapshot.flow):
            speed = math.hypot(velocity_x, velocity_y)
            if speed <= minimum:
                continue
            row, column = divmod(index, snapshot.columns)
            start_x = (
                snapshot.origin_x_m
                + (column + 0.5) * snapshot.resolution_m
            )
            start_y = (
                snapshot.origin_y_m
                + (row + 0.5) * snapshot.resolution_m
            )
            marker = Marker()
            marker.header = source.header
            marker.ns = 'crowd_flow'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points = [
                Point(x=start_x, y=start_y, z=0.05),
                Point(
                    x=start_x + velocity_x,
                    y=start_y + velocity_y,
                    z=0.05,
                ),
            ]
            marker.scale.x = 0.05
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 0.2
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.9
            result.markers.append(marker)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CrowdFieldDiagnosticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
