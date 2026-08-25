"""SemaFORR module overview.

Summary:
    This file exercises test baseline recorder contract behavior for automated verification and regression testing. It centers on `test_recorder_uses_fixed_simulation_time`, `test_recorder_can_reproduce_sensor_loss_and_capture_node_states`. Its package-relative location is `test/contracts/test_baseline_recorder_contract.py`.

Arguments:
    Not applicable at module scope.

Returns:
    Not applicable at module scope.

Raises:
    Import-time dependency errors may propagate.
"""









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
    """Summary:
        Performs the test recorder uses fixed simulation time operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    source = RECORDER_PATH.read_text(encoding="utf-8")

    assert "TICK_HZ = 20.0" in source
    assert "TICK_PERIOD_S = 1.0 / TICK_HZ" in source
    assert "now - self._last_tick" not in source
    assert "self._tick_count += 1" in source
    assert "self._tick_count * self.TICK_PERIOD_S" in source


def test_recorder_can_reproduce_sensor_loss_and_capture_node_states():
    """Summary:
        Performs the test recorder can reproduce sensor loss and capture node states operation for this subsystem.

    Args:
        None.

    Returns:
        Any

    Raises:
        None documented; dependency failures may propagate.
    """
    source = RECORDER_PATH.read_text(encoding="utf-8")

    assert "--sensor-cutoff" in source
    assert "self._sensor_cutoff" in source
    assert '"/decision_records"' in source
    assert "DecisionRecord" in source
    assert '"/navigation_state"' in source
    assert "NavigationState" in source
    assert '"navigation_states": self._navigation_states' in source

    verifier = TIMEOUT_VERIFIER_PATH.read_text(encoding="utf-8")
    assert '"last velocity command is not zero"' in verifier
    assert '"waiting_for_sensors"' in verifier
    assert 'state.get("failure")' in verifier
