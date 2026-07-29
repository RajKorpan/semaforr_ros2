import os
from pathlib import Path


SOURCE_DIR = Path(
    os.environ.get("SEMAFORR_SOURCE_DIR", Path(__file__).resolve().parents[2])
)
RECORDER_PATH = SOURCE_DIR / "scripts" / "record_baseline.py"
TIMEOUT_VERIFIER_PATH = (
    SOURCE_DIR / "scripts" / "verify_sensor_timeout.py"
)


def test_recorder_uses_fixed_simulation_time():
    source = RECORDER_PATH.read_text(encoding="utf-8")

    assert "TICK_HZ = 20.0" in source
    assert "TICK_PERIOD_S = 1.0 / TICK_HZ" in source
    assert "now - self._last_tick" not in source
    assert "self._tick_count += 1" in source
    assert "self._tick_count * self.TICK_PERIOD_S" in source


def test_recorder_can_reproduce_sensor_loss_and_capture_node_states():
    source = RECORDER_PATH.read_text(encoding="utf-8")

    assert "--sensor-cutoff" in source
    assert "self._sensor_cutoff" in source
    assert '"/navigation_state"' in source
    assert '"navigation_states": self._navigation_states' in source

    verifier = TIMEOUT_VERIFIER_PATH.read_text(encoding="utf-8")
    assert '"last velocity command is not zero"' in verifier
    assert '"WaitingForSensors:sensor_"' in verifier
