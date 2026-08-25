"""SemaFORR module overview.

Summary:
    This file exercises test map modes contract behavior for automated verification and regression testing. It centers on `read`, `test_simulator_and_robot_map_controls_are_independent`, `test_simulator_geometry_is_owned_only_by_simulator_fixture`, `test_mapless_defaults_and_installed_examples_are_declared`, `test_static_and_learned_representations_are_separate`. Its package-relative location is `test/contracts/test_map_modes_contract.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









from pathlib import Path


ROOT = Path(__file__).parents[2]


def read(relative):
    """Summary:
        Reads package content for this subsystem.

    Args:
        relative (Any): Supplies relative input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    return (ROOT / relative).read_text(encoding="utf-8")


def test_simulator_and_robot_map_controls_are_independent():
    """Summary:
        Performs the test simulator and robot map controls are independent operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    launch = read("launch/example_simulation.launch.py")
    assert '"simulator_environment_map"' in launch
    assert '"semaforr_map_mode"' in launch
    assert '"semaforr_map_path"' in launch
    assert '"map_based_planning"' in launch
    assert '"map_visualization"' in launch
    assert '"--environment-map"' in launch
    assert '"map.mode": robot_map_mode.perform(context)' in launch


def test_simulator_geometry_is_owned_only_by_simulator_fixture():
    """Summary:
        Performs the test simulator geometry is owned only by simulator fixture operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    simulator = read("scripts/record_baseline.py")
    adapter = read("src/ros/navigation_engine_adapter.cpp")
    assert "_load_environment_walls(environment_map)" in simulator
    assert "simulator_environment_map" in simulator
    assert "environment_map" not in adapter


def test_mapless_defaults_and_installed_examples_are_declared():
    """Summary:
        Performs the test mapless defaults and installed examples are declared operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    configuration = read("config/semaforr.yaml")
    examples = (ROOT.parent / "examples/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert "mode: mapless" in configuration
    assert "enabled: [skeleton]" in configuration
    assert "DIRECTORY core" in examples


def test_static_and_learned_representations_are_separate():
    """Summary:
        Performs the test static and learned representations are separate operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    world = read("include/semaforr/domain/world_model.hpp")
    static_map = read("include/semaforr/domain/static_map.hpp")
    assert "SpatialModel spatial" in world
    assert "const StaticMap* static_map" in world
    assert "GeometryProvenance" in static_map
    assert "map_based_planning_available" in static_map
