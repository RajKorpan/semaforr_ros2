import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def test_component_sources_are_explicit_and_complete():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    source_blocks = re.findall(
        r"set\(SEMAFORR_(?:CORE_DOMAIN|DECISION|SPATIAL|PLANNING|ENGINE)_SOURCES"
        r"(?P<body>.*?)\n\)",
        cmake,
        re.DOTALL,
    )

    assert len(source_blocks) == 5
    assert "GLOB" not in cmake

    declared = {
        source
        for source in re.findall(r"src/(?:[\w]+/)+[\w]+\.cpp", cmake)
        if not source.startswith("src/ros/")
    }
    observed = {
        path.relative_to(SOURCE_DIR).as_posix()
        for path in (SOURCE_DIR / "src").rglob("*.cpp")
        if not path.relative_to(SOURCE_DIR).as_posix().startswith("src/ros/")
    }
    assert declared == observed


def test_source_and_header_layout_is_responsibility_based():
    expected_header_areas = {
        "config",
        "core",
        "domain",
        "decision",
        "exploration",
        "navigation",
        "planning",
        "ros",
        "spatial",
    }
    expected_source_areas = {
        "config",
        "core",
        "decision",
        "navigation",
        "ros",
        "spatial",
    }
    include_root = SOURCE_DIR / "include" / "semaforr"
    source_root = SOURCE_DIR / "src"

    assert not list(include_root.glob("*.h"))
    assert not list(include_root.rglob("*.h"))
    assert not list(source_root.glob("*.cpp"))
    assert {
        path.name for path in include_root.iterdir() if path.is_dir()
    } == expected_header_areas
    assert {
        path.name for path in source_root.iterdir() if path.is_dir()
    } == expected_source_areas


def test_domain_and_compatibility_libraries_are_exported():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_library(semaforr_domain SHARED" in cmake
    assert "add_library(semaforr::domain ALIAS semaforr_domain)" in cmake
    assert (
        "set_target_properties(semaforr_domain PROPERTIES EXPORT_NAME domain)"
        in cmake
    )
    assert "add_library(semaforr_core INTERFACE)" in cmake
    assert "add_library(semaforr::core ALIAS semaforr_core)" in cmake
    assert "set_target_properties(semaforr_core PROPERTIES EXPORT_NAME core)" in cmake
    assert "target_link_libraries(semaforr_core INTERFACE semaforr_domain)" in cmake
    for component in ("planning", "advisors"):
        assert f"add_library(semaforr_{component} SHARED" in cmake
        assert f"add_library(semaforr::{component} ALIAS semaforr_{component})" in cmake
        assert (
            f"set_target_properties(semaforr_{component} PROPERTIES "
            f"EXPORT_NAME {component})"
            in cmake
        )
    assert "add_library(semaforr_ros INTERFACE)" in cmake
    assert "add_library(semaforr::ros ALIAS semaforr_ros)" in cmake
    assert "ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)" in cmake


def test_cpp_code_uses_the_generated_cpp_message_api():
    code = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for directory in ("include", "src")
        for path in (SOURCE_DIR / directory).rglob("*")
        if path.suffix in {".h", ".hpp", ".cpp"}
    )

    assert "semaforr__msg__CrowdModel" not in code
    assert "#include <semaforr/msg/crowd_model.h>" not in code
    assert "semaforr::msg::CrowdModel" in code
    assert "crowd_model.hpp" in code


def test_cpp_build_does_not_embed_python():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    code = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for directory in ("include", "src")
        for path in (SOURCE_DIR / directory).rglob("*")
        if path.suffix in {".h", ".hpp", ".cpp"}
    )

    assert "find_package(Python" not in cmake
    assert "find_package(rclpy" not in cmake
    assert "Python.h" not in code
    assert "Py_Initialize" not in code
    assert "Py_Finalize" not in code


def test_manifest_and_cmake_versions_match():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    package = ET.parse(SOURCE_DIR / "package.xml").getroot()
    cmake_version = re.search(
        r"project\(semaforr VERSION ([0-9.]+)", cmake
    )

    assert cmake_version is not None
    assert cmake_version.group(1) == package.findtext("version")
    assert package.find("build_depend[.='message_generation']") is None
