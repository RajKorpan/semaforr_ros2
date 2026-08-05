from pathlib import Path


ROOT = Path(__file__).parents[2]


def read(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def test_simulator_and_robot_map_controls_are_independent():
    launch = read("launch/example_simulation.launch.py")
    assert '"simulator_environment_map"' in launch
    assert '"semaforr_map_mode"' in launch
    assert '"semaforr_map_path"' in launch
    assert '"map_based_planning"' in launch
    assert '"map_visualization"' in launch
    assert '"--environment-map"' in launch
    assert '"map.mode": robot_map_mode.perform(context)' in launch


def test_simulator_geometry_is_owned_only_by_simulator_fixture():
    simulator = read("scripts/record_baseline.py")
    adapter = read("src/ros/navigation_engine_adapter.cpp")
    assert "_load_environment_walls(environment_map)" in simulator
    assert "simulator_environment_map" in simulator
    assert "environment_map" not in adapter


def test_mapless_defaults_and_installed_examples_are_declared():
    configuration = read("config/semaforr.yaml")
    examples = (ROOT.parent / "examples/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert "mode: mapless" in configuration
    assert "enabled: [skeleton]" in configuration
    assert "DIRECTORY core" in examples


def test_static_and_learned_representations_are_separate():
    world = read("include/semaforr/domain/world_model.hpp")
    static_map = read("include/semaforr/domain/static_map.hpp")
    assert "SpatialModel spatial" in world
    assert "const StaticMap* static_map" in world
    assert "GeometryProvenance" in static_map
    assert "map_based_planning_available" in static_map
