import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[1])
)


def test_core_sources_are_explicit_and_complete():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    source_block = re.search(
        r"set\(SEMAFORR_CORE_SOURCES(?P<body>.*?)\n\)", cmake, re.DOTALL
    )

    assert source_block is not None
    assert "GLOB" not in cmake

    declared = set(re.findall(r"src/[\w]+\.cpp", source_block.group("body")))
    observed = {
        path.relative_to(SOURCE_DIR).as_posix()
        for path in (SOURCE_DIR / "src").glob("*.cpp")
        if path.name != "main.cpp"
    }
    assert declared == observed


def test_core_library_is_exported_with_a_stable_name():
    cmake = (SOURCE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_library(semaforr_core SHARED" in cmake
    assert "add_library(semaforr::core ALIAS semaforr_core)" in cmake
    assert "set_target_properties(semaforr_core PROPERTIES EXPORT_NAME core)" in cmake
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
