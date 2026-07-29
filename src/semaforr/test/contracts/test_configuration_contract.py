import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def test_configuration_parsing_is_outside_controller():
    controller = (
        SOURCE_DIR / "src" / "decision" / "Controller.cpp"
    ).read_text(encoding="utf-8")

    for token in (
        "std::ifstream",
        "istream_iterator",
        "initialize_params",
        "initialize_spatial_model",
        "atoi(",
        "atof(",
    ):
        assert token not in controller

    assert "Controller(semaforr::config::Configuration configuration)" in controller
    assert "semaforr::config::loadConfiguration" in controller


def test_ros_entry_point_loads_one_typed_configuration():
    node = (SOURCE_DIR / "src" / "ros" / "SemaFORRNode.cpp").read_text(
        encoding="utf-8"
    )

    assert "config::Configuration controller_configuration" in node
    assert "configurationFromParameters(node_)" in node
    assert (
        "std::make_unique<NavigationEngineAdapter>(" in node
        and "std::move(controller_configuration)" in node
    )
    parameter_adapter = (
        SOURCE_DIR / "src" / "ros" / "ParameterConfiguration.cpp"
    ).read_text(encoding="utf-8")
    assert "config::loadStructuredConfiguration" in parameter_adapter
    assert "configuration.use_legacy_files" in parameter_adapter
    assert (SOURCE_DIR / "config" / "semaforr.yaml").is_file()


def test_configuration_module_is_typed_and_ros_independent():
    header = (
        SOURCE_DIR / "include" / "semaforr" / "config" / "Configuration.hpp"
    ).read_text(encoding="utf-8")
    implementation = (
        SOURCE_DIR / "src" / "config" / "Configuration.cpp"
    ).read_text(encoding="utf-8")

    for type_name in (
        "ControllerConfiguration",
        "PlannerConfiguration",
        "MapDimensions",
        "AdvisorConfiguration",
        "TaskConfiguration",
        "ConfigurationFiles",
        "Configuration",
    ):
        assert f"struct {type_name}" in header

    assert "std::stod" in implementation
    assert "std::stoll" in implementation
    assert "missing required setting" in implementation
    assert "duplicate setting" in implementation
    assert "unknown setting" in implementation

    combined = header + implementation
    for ros_token in (
        "rclcpp",
        "geometry_msgs",
        "sensor_msgs",
        "semaforr::msg",
    ):
        assert ros_token not in combined
