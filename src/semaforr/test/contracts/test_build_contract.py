import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def cmake_source():
    return (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")


def test_all_production_sources_are_explicit():
    cmake = cmake_source()
    assert "GLOB" not in cmake
    declared = set(re.findall(r"src/(?:[\w]+/)+[\w]+\.cpp", cmake))
    observed = {
        path.relative_to(SOURCE_DIR).as_posix()
        for path in (SOURCE_DIR / "src").rglob("*.cpp")
    }
    assert declared == observed


def test_focused_libraries_are_exported_without_compatibility_targets():
    cmake = cmake_source()
    for component in (
        "domain",
        "planning",
        "exploration",
        "advisors",
        "spatial",
        "navigation",
        "ros_adapters",
    ):
        assert f"add_library(semaforr_{component} SHARED" in cmake
        assert f"add_library(semaforr::{component} ALIAS semaforr_{component})" in cmake
        assert f"EXPORT_NAME {component}" in cmake
    assert "semaforr_core" not in cmake
    assert "add_library(semaforr_ros INTERFACE)" not in cmake
    assert "ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)" in cmake


def test_strict_warnings_apply_to_every_production_library():
    cmake = cmake_source()
    for target in (
        "semaforr_domain",
        "semaforr_planning",
        "semaforr_exploration",
        "semaforr_advisors",
        "semaforr_spatial",
        "semaforr_navigation",
        "semaforr_ros_adapters",
    ):
        assert f"semaforr_apply_project_options({target})" in cmake
        assert f"semaforr_apply_strict_warnings({target})" in cmake


def test_cpp_build_has_no_python_embedding_or_recursive_glob():
    cmake = cmake_source()
    code = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for root in ("include", "src")
        for path in (SOURCE_DIR / root).rglob("*")
        if path.suffix in {".hpp", ".cpp"}
    )
    assert "find_package(Python" not in cmake
    assert "find_package(rclpy" not in cmake
    assert "Python.h" not in code
    assert "Py_Initialize" not in code


def test_manifest_and_cmake_versions_match():
    cmake = cmake_source()
    package = ET.parse(SOURCE_DIR / "package.xml").getroot()
    match = re.search(r"project\(semaforr VERSION ([0-9.]+)", cmake)
    assert match is not None
    assert match.group(1) == package.findtext("version")
