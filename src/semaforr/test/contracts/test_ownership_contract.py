import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def project_code():
    roots = (SOURCE_DIR / "include" / "semaforr", SOURCE_DIR / "src")
    for root in roots:
        for path in root.rglob("*"):
            if path.suffix not in {".hpp", ".cpp"}:
                continue
            if "vendor" in path.parts:
                continue
            yield path


def without_comments_or_literals(source):
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    source = "\n".join(line.split("//", 1)[0] for line in source.splitlines())
    source = re.sub(r'"(?:\\.|[^"\\])*"', '""', source)
    source = re.sub(r"'(?:\\.|[^'\\])*'", "''", source)
    return source


def test_project_code_has_no_manual_allocation_or_deallocation():
    violations = []
    for path in project_code():
        source = without_comments_or_literals(path.read_text(encoding="utf-8"))
        source = re.sub(r"=\s*delete\s*;", "", source)
        tokens = sorted(set(re.findall(r"\b(?:new|delete)\b", source)))
        if tokens:
            violations.append(f"{path}: {', '.join(tokens)}")

    assert not violations, "\n".join(violations)


def test_owners_use_values_or_unique_ptrs():
    controller = (
        SOURCE_DIR / "include" / "semaforr" / "decision" / "Controller.hpp"
    ).read_text(encoding="utf-8")
    agent_state = (
        SOURCE_DIR / "include" / "semaforr" / "decision" / "AgentState.hpp"
    ).read_text(encoding="utf-8")
    graph = (
        SOURCE_DIR / "include" / "semaforr" / "navigation" / "Graph.hpp"
    ).read_text(encoding="utf-8")
    node = (
        SOURCE_DIR / "src" / "ros" / "semaforr_node.cpp"
    ).read_text(encoding="utf-8")
    visualizer = (
        SOURCE_DIR / "include" / "semaforr" / "ros" / "Visualizer.hpp"
    ).read_text(encoding="utf-8")

    assert "std::unique_ptr<Beliefs> beliefs" in controller
    assert "std::vector<std::unique_ptr<PathPlanner>>" in controller
    assert "std::vector<std::unique_ptr<Tier3Advisor>>" in controller
    assert "vector<std::unique_ptr<Task>> owned_tasks" in agent_state
    assert "vector<std::unique_ptr<Node>> ownedNodes" in graph
    assert "vector<std::unique_ptr<Edge>> ownedEdges" in graph
    assert "std::unique_ptr<Controller> controller" in node
    assert "std::unique_ptr<Visualizer> viz_" in node
    assert "std::shared_ptr<rclcpp::Node> node_" not in visualizer
