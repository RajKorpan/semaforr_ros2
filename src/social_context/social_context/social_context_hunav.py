"""Publish HuNav agents through the canonical social observation API."""

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from hunav_msgs.msg import Agents
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from social_context_msgs.msg import (
    PedestrianObservation,
    SocialObservation,
)


def convert_hunav_agents(
    message,
    prediction_steps,
    prediction_step_s,
    confidence,
    position_variance,
):
    """Convert one HuNav agent snapshot into the canonical observation."""
    observation = SocialObservation()
    observation.header = message.header
    observed_at = RclpyTime.from_msg(message.header.stamp)

    for agent in sorted(message.agents, key=lambda item: item.id):
        values = (
            agent.position.position.x,
            agent.position.position.y,
            agent.velocity.linear.x,
            agent.velocity.linear.y,
        )
        if not all(math.isfinite(float(value)) for value in values):
            continue
        pedestrian = PedestrianObservation()
        pedestrian.id = str(agent.id)
        pedestrian.position.x = float(agent.position.position.x)
        pedestrian.position.y = float(agent.position.position.y)
        pedestrian.velocity.x = float(agent.velocity.linear.x)
        pedestrian.velocity.y = float(agent.velocity.linear.y)
        pedestrian.confidence = confidence
        pedestrian.position_covariance = [
            position_variance,
            0.0,
            0.0,
            position_variance,
        ]
        for step in range(1, prediction_steps + 1):
            horizon_s = step * prediction_step_s
            predicted = Point()
            predicted.x = (
                pedestrian.position.x
                + pedestrian.velocity.x * horizon_s
            )
            predicted.y = (
                pedestrian.position.y
                + pedestrian.velocity.y * horizon_s
            )
            pedestrian.predicted_positions.append(predicted)
            stamp = observed_at + Duration(seconds=horizon_s)
            pedestrian.prediction_stamps.append(
                Time(
                    sec=stamp.nanoseconds // 1_000_000_000,
                    nanosec=stamp.nanoseconds % 1_000_000_000,
                )
            )
        observation.pedestrians.append(pedestrian)

    return observation


class SocialContextHuNav(Node):
    """Convert simulator agent state into time-stamped crowd predictions."""

    def __init__(self):
        super().__init__('social_context_hunav')
        self.declare_parameter('agents_topic', '/human_states')
        self.declare_parameter(
            'social_observation_topic',
            '/social_observations',
        )
        self.declare_parameter('prediction_steps', 4)
        self.declare_parameter('prediction_step_s', 0.5)
        self.declare_parameter('confidence', 1.0)
        self.declare_parameter('position_variance', 0.04)

        self._prediction_steps = int(
            self.get_parameter('prediction_steps').value
        )
        self._prediction_step_s = float(
            self.get_parameter('prediction_step_s').value
        )
        self._confidence = float(
            self.get_parameter('confidence').value
        )
        self._variance = float(
            self.get_parameter('position_variance').value
        )
        if self._prediction_steps <= 0:
            raise ValueError('prediction_steps must be positive')
        if (
            not math.isfinite(self._prediction_step_s)
            or self._prediction_step_s <= 0.0
        ):
            raise ValueError(
                'prediction_step_s must be finite and positive'
            )
        if not 0.0 <= self._confidence <= 1.0:
            raise ValueError('confidence must be within [0, 1]')
        if not math.isfinite(self._variance) or self._variance < 0.0:
            raise ValueError(
                'position_variance must be finite and non-negative'
            )

        self._publisher = self.create_publisher(
            SocialObservation,
            self.get_parameter('social_observation_topic').value,
            10,
        )
        self._subscription = self.create_subscription(
            Agents,
            self.get_parameter('agents_topic').value,
            self._callback,
            10,
        )

    def _callback(self, message):
        if not message.header.frame_id:
            self.get_logger().warning(
                'Ignoring HuNav agents without a coordinate frame'
            )
            return
        self._publisher.publish(
            convert_hunav_agents(
                message,
                self._prediction_steps,
                self._prediction_step_s,
                self._confidence,
                self._variance,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = SocialContextHuNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
