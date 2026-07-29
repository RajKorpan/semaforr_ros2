import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
RECORDER_PATH = SOURCE_DIR / "scripts" / "record_baseline.py"


def test_recorder_uses_fixed_simulation_time():
    source = RECORDER_PATH.read_text(encoding="utf-8")

    assert "TICK_HZ = 20.0" in source
    assert "TICK_PERIOD_S = 1.0 / TICK_HZ" in source
    assert "now - self._last_tick" not in source
    assert "self._tick_count += 1" in source
    assert "self._tick_count * self.TICK_PERIOD_S" in source
