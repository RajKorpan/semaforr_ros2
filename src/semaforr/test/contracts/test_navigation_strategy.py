"""Navigation scenario, regression, and quality-gate contracts."""

import importlib.util
import json
import os
from pathlib import Path

SOURCE = Path(os.environ["SEMAFORR_SOURCE_DIR"])


def _comparison_module():
    path = SOURCE / "scripts" / "compare_decision_traces.py"
    specification = importlib.util.spec_from_file_location(
        "compare_decision_traces", path)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader
    specification.loader.exec_module(module)
    return module


def test_every_required_integration_scenario_has_a_fixture():
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
    module_path = SOURCE / "scripts" / "check_source_quality.py"
    specification = importlib.util.spec_from_file_location(
        "check_source_quality", module_path)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader
    specification.loader.exec_module(module)
    manifest = SOURCE / "config" / "quality_gate_manifest.txt"
    assert module.check(module.manifest_paths(SOURCE, manifest)) == []


def test_coverage_gate_rejects_files_below_the_threshold(tmp_path):
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
