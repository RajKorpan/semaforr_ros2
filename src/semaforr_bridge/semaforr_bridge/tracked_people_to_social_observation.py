"""Convert tracker output into the canonical social-navigation observation."""

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from social_context_msgs.msg import (
    PedestrianObservation,
    SocialObservation,
    TrackedPersonArray,
)


def _finite(value):
    return math.isfinite(float(value))


def convert_tracked_people(
    message,
    prediction_steps,
    prediction_step_s,
    history_step_s,
    default_variance,
):
    """Return one validated SocialObservation from tracked people."""
    result = SocialObservation()
    result.header = message.header
    observed_at = RclpyTime.from_msg(message.header.stamp)

    for tracked in sorted(message.people, key=lambda person: person.id):
        values = (
            tracked.x,
            tracked.y,
            tracked.confidence,
        )
        if not all(_finite(value) for value in values):
            continue

        velocity_x = 0.0
        velocity_y = 0.0
        if (
            len(tracked.history_x) >= 2
            and len(tracked.history_x) == len(tracked.history_y)
        ):
            velocity_x = (
                tracked.history_x[-1] - tracked.history_x[-2]
            ) / history_step_s
            velocity_y = (
                tracked.history_y[-1] - tracked.history_y[-2]
            ) / history_step_s

        person = PedestrianObservation()
        person.id = str(tracked.id)
        person.position.x = float(tracked.x)
        person.position.y = float(tracked.y)
        person.velocity.x = float(velocity_x)
        person.velocity.y = float(velocity_y)
        person.confidence = min(1.0, max(0.0, float(tracked.confidence)))
        variance = default_variance / max(person.confidence, 0.05)
        person.position_covariance = [
            variance,
            0.0,
            0.0,
            variance,
        ]

        for step in range(1, prediction_steps + 1):
            horizon_s = prediction_step_s * step
            predicted = Point()
            predicted.x = person.position.x + velocity_x * horizon_s
            predicted.y = person.position.y + velocity_y * horizon_s
            person.predicted_positions.append(predicted)
            stamp = observed_at + Duration(seconds=horizon_s)
            person.prediction_stamps.append(
                Time(sec=stamp.nanoseconds // 1_000_000_000,
                     nanosec=stamp.nanoseconds % 1_000_000_000)
            )
        result.pedestrians.append(person)

    return result


class TrackedPeopleToSocialObservation(Node):
    """Bridge legacy tracker output to the one navigation-facing API."""

    def __init__(self):
        super().__init__('tracked_people_to_social_observation')
        self.declare_parameter(
            'tracked_people_topic',
            '/human_poses_3d_tracked_global',
        )
        self.declare_parameter(
            'social_observation_topic',
            '/social_observations',
        )
        self.declare_parameter('prediction_steps', 4)
        self.declare_parameter('prediction_step_s', 0.5)
        self.declare_parameter('history_step_s', 0.1)
        self.declare_parameter('default_position_variance', 0.09)

        self._prediction_steps = int(
            self.get_parameter('prediction_steps').value
        )
        self._prediction_step_s = float(
            self.get_parameter('prediction_step_s').value
        )
        self._history_step_s = float(
            self.get_parameter('history_step_s').value
        )
        self._default_variance = float(
            self.get_parameter('default_position_variance').value
        )
        if self._prediction_steps <= 0:
            raise ValueError('prediction_steps must be positive')
        for name, value in (
            ('prediction_step_s', self._prediction_step_s),
            ('history_step_s', self._history_step_s),
            ('default_position_variance', self._default_variance),
        ):
            if not _finite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')

        self._publisher = self.create_publisher(
            SocialObservation,
            self.get_parameter('social_observation_topic').value,
            10,
        )
        self._subscription = self.create_subscription(
            TrackedPersonArray,
            self.get_parameter('tracked_people_topic').value,
            self._callback,
            10,
        )

    def _callback(self, message):
        if not message.header.frame_id:
            self.get_logger().warning(
                'Ignoring tracked people without a coordinate frame'
            )
            return
        self._publisher.publish(
            convert_tracked_people(
                message,
                self._prediction_steps,
                self._prediction_step_s,
                self._history_step_s,
                self._default_variance,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackedPeopleToSocialObservation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
