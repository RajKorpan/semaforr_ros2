"""ROS 2 adapter for the incremental crowd-grid model."""

import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from social_context_msgs.msg import SocialObservation
from visualization_msgs.msg import Marker, MarkerArray

from semaforr_crowd.model import CrowdGridModel, PersonSample


class CrowdModelNode(Node):
    """Publish diagnostic grids from the canonical social observation."""

    def __init__(self):
        super().__init__('semaforr_crowd')
        self.declare_parameter('social_topic', 'social_observations')
        self.declare_parameter('density_topic', 'crowd_density')
        self.declare_parameter('risk_topic', 'crowd_risk')
        self.declare_parameter('flow_topic', 'crowd_flow')
        self.declare_parameter('frame', 'map')
        self.declare_parameter('width_m', 200.0)
        self.declare_parameter('height_m', 200.0)
        self.declare_parameter('resolution_m', 1.0)
        self.declare_parameter('origin_x_m', 0.0)
        self.declare_parameter('origin_y_m', 0.0)
        self.declare_parameter('half_life_s', 30.0)
        self.declare_parameter('prediction_half_life_s', 3.0)
        self.declare_parameter('maximum_age_s', 0.75)
        self.declare_parameter('minimum_confidence', 0.25)
        self.declare_parameter('occupancy_scale', 10.0)

        self._frame = self.get_parameter('frame').value
        self._maximum_age_s = float(
            self.get_parameter('maximum_age_s').value
        )
        self._minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        self._occupancy_scale = float(
            self.get_parameter('occupancy_scale').value
        )
        if not self._frame:
            raise ValueError('frame must not be empty')
        if not math.isfinite(self._maximum_age_s):
            raise ValueError('maximum_age_s must be finite')
        if self._maximum_age_s <= 0.0:
            raise ValueError('maximum_age_s must be positive')
        if not 0.0 <= self._minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in [0, 1]')
        if not math.isfinite(self._occupancy_scale):
            raise ValueError('occupancy_scale must be finite')
        if self._occupancy_scale <= 0.0:
            raise ValueError('occupancy_scale must be positive')

        self._model = CrowdGridModel(
            float(self.get_parameter('width_m').value),
            float(self.get_parameter('height_m').value),
            float(self.get_parameter('resolution_m').value),
            float(self.get_parameter('origin_x_m').value),
            float(self.get_parameter('origin_y_m').value),
            float(self.get_parameter('half_life_s').value),
            float(self.get_parameter('prediction_half_life_s').value),
        )
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
            SocialObservation,
            self.get_parameter('social_topic').value,
            self._observe,
            10,
        )

    def _observe(self, message):
        if message.header.frame_id != self._frame:
            self.get_logger().warning(
                'Ignoring social observation in frame %r; expected %r'
                % (message.header.frame_id, self._frame)
            )
            return
        stamp = Time.from_msg(message.header.stamp)
        age_s = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        if age_s < 0.0 or age_s > self._maximum_age_s:
            self.get_logger().warning(
                'Ignoring stale or future social observation'
            )
            return
        people = []
        for pedestrian in message.pedestrians:
            if pedestrian.confidence < self._minimum_confidence:
                continue
            if (
                len(pedestrian.predicted_positions)
                != len(pedestrian.prediction_stamps)
            ):
                self.get_logger().warning(
                    'Ignoring pedestrian %r with mismatched prediction arrays'
                    % pedestrian.id
                )
                continue
            predictions = tuple(
                (
                    point.x,
                    point.y,
                    Time.from_msg(prediction_stamp).nanoseconds / 1.0e9,
                )
                for point, prediction_stamp in zip(
                    pedestrian.predicted_positions,
                    pedestrian.prediction_stamps,
                )
            )
            people.append(PersonSample(
                pedestrian.id,
                pedestrian.position.x,
                pedestrian.position.y,
                pedestrian.velocity.x,
                pedestrian.velocity.y,
                pedestrian.confidence,
                predictions,
            ))
        try:
            self._model.observe(stamp.nanoseconds / 1.0e9, people)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        snapshot = self._model.snapshot()
        self._density_publisher.publish(
            self._occupancy(message, snapshot.density)
        )
        self._risk_publisher.publish(
            self._occupancy(message, snapshot.risk)
        )
        self._flow_publisher.publish(
            self._flow_markers(message, snapshot)
        )

    def _occupancy(self, observation, values):
        message = OccupancyGrid()
        message.header = observation.header
        message.info.resolution = self._model.resolution_m
        message.info.width = self._model.columns
        message.info.height = self._model.rows
        message.info.origin.position.x = self._model.origin_x_m
        message.info.origin.position.y = self._model.origin_y_m
        message.info.origin.orientation.w = 1.0
        message.data = [
            min(100, max(0, round(value * self._occupancy_scale)))
            for value in values
        ]
        return message

    def _flow_markers(self, observation, snapshot):
        result = MarkerArray()
        for index, (velocity_x, velocity_y) in enumerate(zip(
            snapshot.flow_x,
            snapshot.flow_y,
        )):
            speed = math.hypot(velocity_x, velocity_y)
            if speed <= 1.0e-6:
                continue
            row, column = divmod(index, self._model.columns)
            marker = Marker()
            marker.header = observation.header
            marker.ns = 'crowd_flow'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.04
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.a = 0.8
            marker.color.r = 0.1
            marker.color.g = 0.4
            marker.color.b = 1.0
            start = Point()
            start.x = (
                self._model.origin_x_m
                + (column + 0.5) * self._model.resolution_m
            )
            start.y = (
                self._model.origin_y_m
                + (row + 0.5) * self._model.resolution_m
            )
            end = Point()
            end.x = start.x + velocity_x
            end.y = start.y + velocity_y
            marker.points = [start, end]
            result.markers.append(marker)
        return result


def main(args=None):
    """Run the ROS 2 crowd-model adapter."""
    rclpy.init(args=args)
    node = CrowdModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
