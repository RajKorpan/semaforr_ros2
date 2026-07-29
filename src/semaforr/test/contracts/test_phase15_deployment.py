"""Phase 15 documentation and deployment contracts."""

import os
from pathlib import Path

import pytest


SOURCE = Path(os.environ["SEMAFORR_SOURCE_DIR"])
WORKSPACE = SOURCE.parents[1]


def test_maintained_documentation_set_is_complete():
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
