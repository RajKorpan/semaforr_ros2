"""SemaFORR module overview.

Summary:
    This file exercises test navigation strategy behavior for automated verification and regression testing. It centers on `_comparison_module`, `test_every_required_integration_scenario_has_a_fixture`, `test_recorded_stage_tutorial_trace_is_an_exact_match`, `test_changed_decisions_require_a_classification_and_rationale`, `test_refactored_source_quality_manifest_passes`, `test_coverage_gate_rejects_files_below_the_threshold`. Its package-relative location is `test/contracts/test_navigation_strategy.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""










import importlib.util
import json
import os
from pathlib import Path

SOURCE = Path(os.environ["SEMAFORR_SOURCE_DIR"])


def _comparison_module():
    """Summary:
        Performs the comparison module operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    path = SOURCE / "scripts" / "compare_decision_traces.py"
    specification = importlib.util.spec_from_file_location(
        "compare_decision_traces", path)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader
    specification.loader.exec_module(module)
    return module


def test_every_required_integration_scenario_has_a_fixture():
    """Summary:
        Performs the test every required integration scenario has a fixture operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    document = json.loads(
        (SOURCE / "test" / "fixtures" / "scenarios" /
         "navigation_scenarios.json").read_text(encoding="utf-8"))
    names = {scenario["name"] for scenario in document["scenarios"]}
    assert names == {
        "empty_corridor",
        "doorway",
        "obstacle_ahead",
        "dead_end",
        "multiple_targets",
        "stuck_robot",
        "pedestrian_crossing",
        "dense_crowd",
        "sensor_timeout",
        "hallway_network",
        "highway_crossing",
        "large_room",
        "dynamic_obstacle",
        "failed_movement",
        "negative_coordinate_map",
    }


def test_recorded_stage_tutorial_trace_is_an_exact_match():
    """Summary:
        Performs the test recorded stage tutorial trace is an exact match operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    compare = _comparison_module().compare
    baseline = json.loads(
        (SOURCE / "test" / "fixtures" / "baseline" /
         "stage_tutorial.expected.json").read_text(encoding="utf-8"))
    current = json.loads(
        (SOURCE / "test" / "fixtures" / "regression" /
         "stage_tutorial.actual.json").read_text(encoding="utf-8"))
    annotations = json.loads(
        (SOURCE / "test" / "fixtures" / "regression" /
         "stage_tutorial.classifications.json").read_text(encoding="utf-8"))

    report = compare(baseline, current, annotations)
    assert report["errors"] == []
    assert report["counts"] == {"exact_match": 11}


def test_changed_decisions_require_a_classification_and_rationale():
    """Summary:
        Performs the test changed decisions require a classification and rationale operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    compare = _comparison_module().compare
    baseline = {
        "decisions": [
            {
                "task": 0,
                "decision": 1,
                "chosen_action": [0, 1],
                "decision_tier": 3.0,
                "chosen_planner": "skeleton",
            }
        ]
    }
    current = {
        "decisions": [
            {
                "task": 0,
                "decision": 1,
                "chosen_action": [3, 0],
                "decision_tier": "safe_stop",
                "chosen_planner": "skeleton",
            }
        ]
    }
    assert compare(baseline, current, {})["errors"]
    report = compare(
        baseline,
        current,
        {
            "0:1": {
                "classification": "acceptable_intentional_improvement",
                "rationale": "No candidate survived; safe stop replaces crash.",
            }
        },
    )
    assert report["errors"] == []
    assert report["counts"] == {
        "acceptable_intentional_improvement": 1}

    regression = compare(
        baseline,
        current,
        {
            "0:1": {
                "classification": "regression_requiring_correction",
                "rationale": "Unexpected mission divergence.",
            }
        },
    )
    assert "regression requires correction" in regression["errors"][0]


def test_refactored_source_quality_manifest_passes():
    """Summary:
        Performs the test refactored source quality manifest passes operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    module_path = SOURCE / "scripts" / "check_source_quality.py"
    specification = importlib.util.spec_from_file_location(
        "check_source_quality", module_path)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader
    specification.loader.exec_module(module)
    manifest = SOURCE / "config" / "quality_gate_manifest.txt"
    assert module.check(module.manifest_paths(SOURCE, manifest)) == []


def test_coverage_gate_rejects_files_below_the_threshold(tmp_path):
    """Summary:
        Performs the test coverage gate rejects files below the threshold operation for this subsystem.

    Args:
        tmp_path (Any): Supplies tmp path input to the operation.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    module_path = SOURCE / "scripts" / "check_coverage.py"
    specification = importlib.util.spec_from_file_location(
        "check_coverage", module_path)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader
    specification.loader.exec_module(module)
    fixture = tmp_path / "coverage.info"
    fixture.write_text(
        "SF:/workspace/src/example.cpp\n"
        "DA:1,1\nDA:2,0\nend_of_record\n",
        encoding="utf-8")
    coverage = module.parse_lcov(fixture)
    errors, percentages = module.evaluate(
        coverage,
        {
            "minimum_line_percent": 80.0,
            "files": ["src/example.cpp"],
        },
    )
    assert percentages["src/example.cpp"] == 50.0
    assert errors == ["src/example.cpp: 50.0% is below 80.0%"]
