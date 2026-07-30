import os
from pathlib import Path
import re


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)


def production_code():
    return "\n".join(
        path.read_text(encoding="utf-8")
        for root in (SOURCE_DIR / "include", SOURCE_DIR / "src")
        for path in root.rglob("*")
        if path.suffix in {".hpp", ".cpp"}
    )


def test_no_explicit_heap_allocation_or_raw_owning_delete():
    code = production_code()
    assert re.search(r"\bnew\s+[A-Za-z_:][A-Za-z0-9_:<>]*\s*(?:\(|\[)", code) is None
    assert re.search(r"\bdelete\s+[A-Za-z_]", code) is None


def test_polymorphic_owners_use_unique_ptr_and_virtual_destructors():
    code = production_code()
    assert "std::unique_ptr<Advisor>" in code
    assert "std::unique_ptr<Planner>" in code
    assert "std::unique_ptr<SpatialLearner>" in code
    for header in (
        "include/semaforr/decision/advisor.hpp",
        "include/semaforr/planning/planner.hpp",
        "include/semaforr/spatial/spatial_learner.hpp",
    ):
        assert "virtual ~" in (SOURCE_DIR / header).read_text(encoding="utf-8")


def test_no_shared_ptr_outside_ros_adapter_layer():
    violations = []
    for root in (SOURCE_DIR / "include/semaforr", SOURCE_DIR / "src"):
        for path in root.rglob("*"):
            if path.suffix not in {".hpp", ".cpp"} or "/ros/" in path.as_posix():
                continue
            if "shared_ptr" in path.read_text(encoding="utf-8"):
                violations.append(path.relative_to(SOURCE_DIR).as_posix())
    assert not violations
