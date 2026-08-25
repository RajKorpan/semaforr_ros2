"""SemaFORR module overview.

Summary:
    This file exercises test simulator map behavior for automated verification and regression testing. It centers on `test_simulator_loads_environment_geometry_independently`, `test_simulator_raycast_observes_wall_and_motion_segment_is_blocked`. Its package-relative location is `test/unit/test_simulator_map.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









import importlib.util
from pathlib import Path


ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location(
    "record_baseline", ROOT / "scripts/record_baseline.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_simulator_loads_environment_geometry_independently():
    """Summary:
        Performs the test simulator loads environment geometry independently operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    walls = MODULE._load_environment_walls(
        ROOT / "test/fixtures/maps/negative.xml"
    )
    assert walls == [((0.0, -4.0), (0.0, 4.0))]


def test_simulator_raycast_observes_wall_and_motion_segment_is_blocked():
    """Summary:
        Performs the test simulator raycast observes wall and motion segment is blocked operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    wall = ((0.0, -4.0), (0.0, 4.0))
    assert (
        MODULE._segment_intersection_distance(
            (-2.0, 0.0), 0.0, 5.0, wall
        )
        == 2.0
    )
    assert (
        MODULE._segment_intersection_distance(
            (-2.0, 0.0), 0.0, 1.0, wall
        )
        is None
    )
