import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def test_uniform_spatial_learner_lifecycle_is_public_and_ros_independent():
    header = (
        SOURCE_DIR
        / "include"
        / "semaforr"
        / "spatial"
        / "learner.hpp"
    ).read_text(encoding="utf-8")

    assert "class SpatialLearner" in header
    assert "virtual void observe(const NavigationEpisode& episode) = 0;" in header
    assert "virtual void rebuild() = 0;" in header
    assert "virtual SpatialModelUpdate snapshot() const = 0;" in header
    assert "virtual ~SpatialLearner() = default;" in header
    assert "rclcpp" not in header
    assert "_msgs/" not in header


def test_each_representation_has_a_focused_module():
    include_dir = SOURCE_DIR / "include" / "semaforr" / "spatial" / "learners"
    source_dir = SOURCE_DIR / "src" / "spatial"
    modules = {
        "trail": "Trail",
        "conveyor": "Conveyor",
        "region": "Region",
        "door_exit": "DoorExit",
        "hallway": "Hallway",
        "barrier": "Barrier",
        "passage_skeleton": "PassageSkeleton",
    }
    for filename, class_name in modules.items():
        header = (include_dir / f"{filename}_learner.hpp").read_text(
            encoding="utf-8"
        )
        source = (source_dir / f"{filename}_learner.cpp").read_text(
            encoding="utf-8"
        )
        assert f"class {class_name}Learner final" in header
        assert f"{class_name}Learner::" in source


def test_coordinator_owns_learners_and_exposes_independent_controls():
    header = (
        SOURCE_DIR
        / "include"
        / "semaforr"
        / "spatial"
        / "spatial_learning_coordinator.hpp"
    ).read_text(encoding="utf-8")
    source = (
        SOURCE_DIR / "src" / "spatial" / "spatial_learning_coordinator.cpp"
    ).read_text(encoding="utf-8")

    for operation in (
        "void addLearner(",
        "void setEnabled(",
        "void rebuild(",
        "std::optional<SpatialModelUpdate> snapshot(",
        "std::vector<LearnerInspection> inspect() const",
        "std::string serialize(",
        "void applyTo(",
    ):
        assert operation in header
    assert "std::unique_ptr<SpatialLearner>" in header
    assert "LearningStep" not in header
    assert "if (!update.usable())" in source


def test_spatial_component_is_an_exported_library():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_library(semaforr_spatial SHARED" in cmake
    assert "add_library(semaforr::spatial ALIAS semaforr_spatial)" in cmake
    assert (
        "set_target_properties(semaforr_spatial PROPERTIES EXPORT_NAME spatial)"
        in cmake
    )
