import math

import pytest
from hunav_msgs.msg import Agent, Agents

from social_context.social_context_hunav import convert_hunav_agents


def agent(identifier, x, y, velocity_x, velocity_y):
    value = Agent()
    value.id = identifier
    value.position.position.x = x
    value.position.position.y = y
    value.velocity.linear.x = velocity_x
    value.velocity.linear.y = velocity_y
    return value


def test_hunav_conversion_is_stable_and_time_stamped():
    message = Agents()
    message.header.frame_id = "map"
    message.header.stamp.sec = 20
    message.agents = [
        agent(7, 2.0, 1.0, -0.5, 0.0),
        agent(2, 0.0, 1.0, 0.25, -0.5),
    ]

    result = convert_hunav_agents(message, 2, 0.5, 0.9, 0.04)

    assert [person.id for person in result.pedestrians] == ["2", "7"]
    pedestrian = result.pedestrians[0]
    assert pedestrian.predicted_positions[1].x == pytest.approx(0.25)
    assert pedestrian.predicted_positions[1].y == pytest.approx(0.5)
    assert pedestrian.prediction_stamps[1].sec == 21
    assert pedestrian.confidence == pytest.approx(0.9)
    assert pedestrian.position_covariance == pytest.approx(
        [0.04, 0.0, 0.0, 0.04]
    )


def test_hunav_conversion_drops_non_finite_agents():
    message = Agents()
    message.header.frame_id = "map"
    message.agents = [agent(1, math.inf, 0.0, 0.0, 0.0)]

    result = convert_hunav_agents(message, 1, 0.5, 1.0, 0.04)

    assert not result.pedestrians
