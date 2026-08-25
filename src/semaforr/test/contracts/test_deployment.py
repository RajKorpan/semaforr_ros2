"""SemaFORR module overview.

Summary:
    This file exercises test deployment behavior for automated verification and regression testing. It centers on `test_maintained_documentation_set_is_complete`, `test_example_launch_resolves_only_installed_assets`, `test_runtime_assets_are_installed_and_declared`, `test_container_builds_workspace_and_runs_installed_example`, `test_ci_has_build_lint_unit_integration_and_image_gates`. Its package-relative location is `test/contracts/test_deployment.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""










import os
from pathlib import Path

import pytest


SOURCE = Path(os.environ["SEMAFORR_SOURCE_DIR"])
WORKSPACE = SOURCE.parents[1]


def test_maintained_documentation_set_is_complete():
    """Summary:
        Performs the test maintained documentation set is complete operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    required = {
        "README.md",
        "architecture.md",
        "decision-tiers.md",
        "advisor-catalog.md",
        "planner-catalog.md",
        "configuration-reference.md",
        "topics-and-frames.md",
        "spatial-learning.md",
        "troubleshooting.md",
        "contributing.md",
        "legacy-configuration-migration.md",
        "deployment.md",
    }
    assert required <= {path.name for path in (SOURCE / "docs").glob("*.md")}


def test_example_launch_resolves_only_installed_assets():
    """Summary:
        Performs the test example launch resolves only installed assets operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    launch = (
        SOURCE / "launch" / "example_simulation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'get_package_share_directory("semaforr")' in launch
    assert "config/semaforr.yaml" not in launch
    assert "src/semaforr" not in launch
    assert "open_room.xml" in launch
    assert "mission.conf" in launch
    assert "semaforr_record_baseline" in launch
    assert "semaforr.rviz" in launch


def test_runtime_assets_are_installed_and_declared():
    """Summary:
        Performs the test runtime assets are installed and declared operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    cmake = (SOURCE / "CMakeLists.txt").read_text(encoding="utf-8")
    manifest = (SOURCE / "package.xml").read_text(encoding="utf-8")
    assert "DIRECTORY config docs launch rviz" in cmake
    assert "<exec_depend>rviz2</exec_depend>" in manifest
    assert (SOURCE / "config" / "semaforr.yaml").is_file()
    assert (
        SOURCE / "config" / "stage_tutorial" / "stage_tutorialS.xml"
    ).is_file()
    assert (SOURCE / "config" / "stage_tutorial" / "target.conf").is_file()
    assert (SOURCE / "config" / "example" / "open_room.xml").is_file()
    assert (SOURCE / "config" / "example" / "mission.conf").is_file()
    assert (SOURCE / "rviz" / "semaforr.rviz").is_file()


def test_container_builds_workspace_and_runs_installed_example():
    """Summary:
        Performs the test container builds workspace and runs installed example operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    if not (WORKSPACE / "Dockerfile").is_file():
        pytest.skip("package-only source copy does not include workspace files")
    dockerfile = (WORKSPACE / "Dockerfile").read_text(encoding="utf-8")
    compose = (WORKSPACE / "docker-compose.yml").read_text(encoding="utf-8")
    entrypoint = (
        WORKSPACE / "docker" / "entrypoint.sh"
    ).read_text(encoding="utf-8")
    assert "COPY src ./src" in dockerfile
    assert "rosdep install --from-paths src --ignore-src" in dockerfile
    assert "colcon build" in dockerfile
    assert 'source /workspace/install/setup.bash' in entrypoint
    assert "example_simulation.launch.py" in compose
    assert ".:/workspace" not in compose


def test_ci_has_build_lint_unit_integration_and_image_gates():
    """Summary:
        Performs the test ci has build lint unit integration and image gates operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    if not (WORKSPACE / ".github").is_dir():
        pytest.skip("package-only source copy does not include CI files")
    workflow = (
        WORKSPACE / ".github" / "workflows" / "ros2-humble.yml"
    ).read_text(encoding="utf-8")
    for expected in (
        "colcon build",
        "check_source_quality.py",
        "Unit and contract tests",
        "Integration tests",
        "example_simulation.launch.py",
        "docker build",
    ):
        assert expected in workflow
