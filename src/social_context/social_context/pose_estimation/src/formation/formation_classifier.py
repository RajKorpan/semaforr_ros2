#!/usr/bin/env python3
"""
Classification logic for F-formations (proxemics groupings), adapted
from Alan's code at https://github.com/asianwaffles/Ssemafor-FformationDetection.

Ported: pairwise/group classification (vis-a-vis, side-by-side, L-shaped,
circular) using real body orientation and floor-plane positions.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List

import numpy as np


class FormationType(Enum):
    VIS_A_VIS = "vis-a-vis"
    L_SHAPED = "L-shaped"
    SIDE_BY_SIDE = "side-by-side"
    CIRCULAR = "circular"
    NONE = "none"


@dataclass
class Person:
    """One person's floor-plane position + heading."""
    id: int
    x: float
    y: float
    yaw: float  # radians, atan2(y, x) convention (matches TrackedPerson orientation)


def angle_diff_deg(a: float, b: float) -> float:
    """Smallest absolute difference between two angles (degrees)."""
    diff = abs(a - b) % 360
    return min(diff, 360 - diff)


def is_facing(body_yaw_deg: float, target_yaw_deg: float, angle_threshold_deg: float) -> bool:
    return angle_diff_deg(body_yaw_deg, target_yaw_deg) < angle_threshold_deg


def classify_pair(people: List[Person], distance_threshold: float, angle_threshold_deg: float = 60.0) -> FormationType:
    """Classify a 2-person cluster."""
    p1, p2 = people
    d = math.hypot(p2.x - p1.x, p2.y - p1.y)
    if d >= distance_threshold:
        return FormationType.NONE

    interaction_angle = math.degrees(math.atan2(p2.y - p1.y, p2.x - p1.x))
    yaw1_deg = math.degrees(p1.yaw)
    yaw2_deg = math.degrees(p2.yaw)

    p1_facing = is_facing(yaw1_deg, interaction_angle, angle_threshold_deg)
    p2_facing = is_facing(yaw2_deg, interaction_angle + 180, angle_threshold_deg)

    yaw_diff = angle_diff_deg(yaw1_deg, yaw2_deg)

    if p1_facing and p2_facing:
        return FormationType.VIS_A_VIS
    elif yaw_diff < 45:
        return FormationType.SIDE_BY_SIDE
    elif 45 <= yaw_diff <= 135:
        return FormationType.L_SHAPED
    else:
        return FormationType.SIDE_BY_SIDE


def classify_group(people: List[Person], distance_threshold: float, angle_threshold_deg: float = 60.0) -> FormationType:
    """Classify a 3+-person cluster. Only distinguishes circular vs.
    side-by-side -- the source repo doesn't attempt finer group shapes
    (e.g. L-shaped) for 3+ people either, so this keeps parity with it."""
    positions = np.array([[p.x, p.y] for p in people])
    n = len(people)
    dists = np.linalg.norm(positions[:, None] - positions[None, :], axis=2)
    max_distance = np.max(dists[dists > 0]) if n > 1 else 0.0
    if max_distance >= distance_threshold:
        return FormationType.NONE

    center = positions.mean(axis=0)
    dists_to_center = np.linalg.norm(positions - center, axis=1)
    std_dist = np.std(dists_to_center)
    mean_dist = np.mean(dists_to_center)

    # Circular: roughly equidistant from center + mostly facing inward
    if std_dist < mean_dist * 0.4:
        facing_count = 0
        for i, p in enumerate(people):
            to_center = center - positions[i]
            target_yaw_deg = math.degrees(math.atan2(to_center[1], to_center[0]))
            if is_facing(math.degrees(p.yaw), target_yaw_deg, angle_threshold_deg):
                facing_count += 1
        if facing_count >= n * 0.6:
            return FormationType.CIRCULAR

    return FormationType.SIDE_BY_SIDE


def classify_cluster(people: List[Person], distance_threshold: float, angle_threshold_deg: float = 60.0) -> FormationType:
    """Dispatch to pair/group classification by cluster size."""
    if len(people) < 2:
        return FormationType.NONE
    elif len(people) == 2:
        return classify_pair(people, distance_threshold, angle_threshold_deg)
    else:
        return classify_group(people, distance_threshold, angle_threshold_deg)
