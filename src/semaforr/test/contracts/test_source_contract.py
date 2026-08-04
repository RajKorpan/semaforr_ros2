import json
import math
import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
CONTRACT_PATH = (
    SOURCE_DIR / "test" / "fixtures" / "baseline" / "source_contract.json"
)


def load_contract():
    return json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))


def parse_configuration(path):
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        values[fields[0]] = fields[1:]
    return values


def extract_number(source, pattern):
    match = re.search(pattern, source)
    assert match is not None, f"source pattern was not found: {pattern}"
    return float(match.group(1))


def extract_yaml_number(source, key):
    match = re.search(
        rf"^\s*{re.escape(key)}:\s*([0-9.]+)\s*$",
        source,
        re.MULTILINE,
    )
    assert match is not None, f"YAML setting was not found: {key}"
    return float(match.group(1))


def test_ros_topic_contract():
    contract = load_contract()["ros"]
    source = (
        SOURCE_DIR / "src" / "ros" / "semaforr_node_component.cpp"
    ).read_text(encoding="utf-8")
    yaml = (SOURCE_DIR / "config" / "semaforr.yaml").read_text(
        encoding="utf-8"
    )

    assert f'rclcpp::Node("{contract["node_name"]}", options)' in source
    for topic, message_type in contract["subscriptions"].items():
        assert f"create_subscription<{message_type}>" in source
        assert re.search(rf"^\s*\w+:\s*{re.escape(topic)}\s*$", yaml, re.MULTILINE)

    publisher = contract["publisher"]
    topic, message_type = next(iter(publisher.items()))
    assert f"create_publisher<{message_type}>" in source
    assert re.search(rf"^\s*\w+:\s*{re.escape(topic)}\s*$", yaml, re.MULTILINE)


def test_motion_command_contract():
    contract = load_contract()["command_velocities"]
    yaml = (SOURCE_DIR / "config" / "semaforr.yaml").read_text(
        encoding="utf-8"
    )
    observed = {
        "FORWARD": {
            "linear_x": extract_yaml_number(yaml, "linear_velocity_mps"),
            "angular_z": 0.0,
        },
        "RIGHT_TURN": {
            "linear_x": extract_yaml_number(yaml, "turn_linear_velocity_mps"),
            "angular_z": -extract_yaml_number(yaml, "angular_velocity_radps"),
        },
        "LEFT_TURN": {
            "linear_x": extract_yaml_number(yaml, "turn_linear_velocity_mps"),
            "angular_z": extract_yaml_number(yaml, "angular_velocity_radps"),
        },
        "PAUSE": {"linear_x": 0.0, "angular_z": 0.0},
    }
    assert observed == contract


def test_action_completion_contract():
    contract = load_contract()["completion"]
    yaml = (SOURCE_DIR / "config" / "semaforr.yaml").read_text(
        encoding="utf-8"
    )
    observed = {
        key: extract_yaml_number(yaml, key)
        for key in contract
    }
    assert observed == contract


def test_default_configuration_contract():
    contract = load_contract()["configuration"]
    config = parse_configuration(SOURCE_DIR / "config" / "params.conf")

    assert [float(value) for value in config["move"]] == contract["move_m"]
    assert [float(value) for value in config["rotate"]] == contract["rotate_rad"]
    assert int(config["decisionlimit"][0]) == contract["decisionlimit"]

    for key, expected in contract["features"].items():
        assert int(config[key][0]) == expected


def test_tutorial_target_contract():
    contract = load_contract()["tutorial_targets"]
    target_path = SOURCE_DIR / "config" / "stage_tutorial" / "target.conf"
    observed = [
        [float(value) for value in line.split()]
        for line in target_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]

    assert len(observed) == len(contract)
    for observed_target, expected_target in zip(observed, contract):
        assert len(observed_target) == 2
        assert all(
            math.isclose(observed_value, expected_value)
            for observed_value, expected_value in zip(
                observed_target, expected_target
            )
        )
