import math

import pytest
from social_context_msgs.msg import TrackedPerson, TrackedPersonArray

from semaforr_bridge.tracked_people_to_social_observation import (
    convert_tracked_people,
)


def tracked_person(identifier, x, y, confidence=0.8):
    person = TrackedPerson()
    person.id = identifier
    person.x = x
    person.y = y
    person.confidence = confidence
    person.history_x = [x - 0.1, x]
    person.history_y = [y, y]
    return person


def test_tracker_conversion_preserves_identity_and_predicts():
    message = TrackedPersonArray()
    message.header.frame_id = "map"
    message.header.stamp.sec = 10
    message.people = [
        tracked_person(9, 2.0, 1.0),
        tracked_person(3, 1.0, 0.0),
    ]

    result = convert_tracked_people(message, 2, 0.5, 0.1, 0.09)

    assert result.header.frame_id == "map"
    assert [person.id for person in result.pedestrians] == ["3", "9"]
    pedestrian = result.pedestrians[0]
    assert pedestrian.velocity.x == pytest.approx(1.0)
    assert pedestrian.predicted_positions[0].x == pytest.approx(1.5)
    assert pedestrian.prediction_stamps[0].sec == 10
    assert pedestrian.prediction_stamps[0].nanosec == 500_000_000
    assert pedestrian.position_covariance[0] > 0.0


def test_tracker_conversion_drops_non_finite_people():
    message = TrackedPersonArray()
    message.header.frame_id = "map"
    message.people = [tracked_person(1, math.nan, 0.0)]

    result = convert_tracked_people(message, 1, 0.5, 0.1, 0.09)

    assert not result.pedestrians
