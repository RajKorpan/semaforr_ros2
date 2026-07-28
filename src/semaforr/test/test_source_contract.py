import json
import math
import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[1])
)
CONTRACT_PATH = SOURCE_DIR / "test" / "baseline" / "source_contract.json"


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


def test_ros_topic_contract():
    contract = load_contract()["ros"]
    source = (
        SOURCE_DIR / "src" / "ros" / "semaforr_node.cpp"
    ).read_text(encoding="utf-8")

    assert f'Node("{contract["node_name"]}")' in source
    for topic, message_type in contract["subscriptions"].items():
        pattern = (
            rf"create_subscription<{re.escape(message_type)}>\s*\(\s*"
            rf'"{re.escape(topic)}"'
        )
        assert re.search(pattern, source)

    publisher = contract["publisher"]
    topic, message_type = next(iter(publisher.items()))
    pattern = (
        rf"create_publisher<{re.escape(message_type)}>\s*\(\s*"
        rf'"{re.escape(topic)}"'
    )
    assert re.search(pattern, source)


def test_motion_command_contract():
    contract = load_contract()["command_velocities"]
    source = (
        SOURCE_DIR / "src" / "ros" / "semaforr_node.cpp"
    ).read_text(encoding="utf-8")
    source = source[source.index(
        "geometry_msgs::msg::Twist convert_to_vel"
    ):]

    blocks = {
        name: re.search(
            rf"(?:if|else if)\s*\(\s*action\.type\s*==\s*{name}\s*\)"
            rf"\s*\{{(?P<body>.*?)\n\s*\}}",
            source,
            re.DOTALL,
        )
        for name in contract
    }
    for name, match in blocks.items():
        assert match is not None, f"command block for {name} was not found"
        body = match.group("body")
        expected = contract[name]
        if name != "PAUSE":
            assert extract_number(
                body, r"base_cmd\.linear\.x\s*=\s*(-?[0-9.]+)"
            ) == expected["linear_x"]
        if name in ("RIGHT_TURN", "LEFT_TURN"):
            angular_assignments = re.findall(
                r"^\s*base_cmd\.angular\.z\s*=\s*(-?[0-9.]+)",
                body,
                re.MULTILINE,
            )
            assert angular_assignments
            assert float(angular_assignments[-1]) == expected["angular_z"]


def test_action_completion_contract():
    contract = load_contract()["completion"]
    source = (
        SOURCE_DIR / "src" / "ros" / "semaforr_node.cpp"
    ).read_text(encoding="utf-8")

    observed = {
        "loop_rate_hz": extract_number(
            source, r"rclcpp::Rate\s+rate\(\s*([0-9.]+)\s*\)"
        ),
        "move_epsilon_m": extract_number(
            source, r"epsilon_move\s*=\s*([0-9.]+)"
        ),
        "turn_epsilon_rad": extract_number(
            source, r"epsilon_turn\s*=\s*([0-9.]+)"
        ),
        "forward_timeout_multiplier": extract_number(
            source, r"elapsed_time\s*>=\s*([0-9.]+)\s*\*\s*expected_travel"
        ),
        "turn_timeout_multiplier": extract_number(
            source, r"elapsed_time\s*>=\s*turn_expected\s*\*\s*([0-9.]+)"
        ),
        "pause_timeout_s": extract_number(
            source,
            r"action\.type\s*==\s*PAUSE\)\s*or\s*\(elapsed_time\s*>=\s*([0-9.]+)",
        ),
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
