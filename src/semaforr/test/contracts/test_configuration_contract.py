import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def test_runtime_configuration_is_parameter_only():
    adapter = (SOURCE_DIR / "src/ros/parameter_configuration.cpp").read_text(
        encoding="utf-8"
    )
    yaml = (SOURCE_DIR / "config/semaforr.yaml").read_text(encoding="utf-8")
    for obsolete in (
        "configuration.use_legacy_files",
        "configuration.legacy.",
        "semaforr_path",
        "target_set",
        "map_config",
        "map_dimensions",
    ):
        assert obsolete not in adapter
        assert obsolete not in yaml
    assert "map.path: required path is empty" in adapter
    assert "mission.tasks_path: required path is empty" in adapter


def test_structured_configuration_validates_all_runtime_invariants():
    source = (SOURCE_DIR / "src/config/navigation_configuration.cpp").read_text(
        encoding="utf-8"
    )
    for diagnostic in (
        "must not be empty",
        "must be finite",
        "must be positive",
        "unknown advisor",
        "planner",
        "map",
    ):
        assert diagnostic in source


def test_legacy_converter_is_offline_only():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "convert_legacy_config.py" in cmake
    assert "loadConfiguration({" not in (
        SOURCE_DIR / "src/ros/parameter_configuration.cpp"
    ).read_text(encoding="utf-8")
