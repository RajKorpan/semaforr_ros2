import importlib.util
from pathlib import Path


ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location(
    "record_baseline", ROOT / "scripts/record_baseline.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_simulator_loads_environment_geometry_independently():
    walls = MODULE._load_environment_walls(
        ROOT / "test/fixtures/maps/negative.xml"
    )
    assert walls == [((0.0, -4.0), (0.0, 4.0))]


def test_simulator_raycast_observes_wall_and_motion_segment_is_blocked():
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
