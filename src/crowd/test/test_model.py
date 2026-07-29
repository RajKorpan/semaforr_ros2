from types import SimpleNamespace

import pytest

from semaforr_crowd.model import CrowdFieldCell, CrowdFieldSnapshot


def cell(direction=0, density=0.25, risk=0.1):
    flow = [0.0] * 8
    flow[direction] = 0.5
    return CrowdFieldCell(
        density=density,
        learned_encounter_risk=risk,
        directional_flow=tuple(flow),
        visibility_exposures=4.0,
        pedestrian_hits=1.0,
        risk_encounters=1.0,
        risk_experiences=10.0,
        confidence=0.5,
    )


def test_snapshot_preserves_evidence_and_projects_eight_bin_flow():
    snapshot = CrowdFieldSnapshot(
        frame_id='map',
        width_m=2.0,
        height_m=1.0,
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        columns=2,
        rows=1,
        estimator='count_exposure',
        version=1,
        cells=(cell(0), cell(2, density=0.5, risk=0.2)),
    )
    snapshot.validate()

    assert snapshot.density == pytest.approx((0.25, 0.5))
    assert snapshot.risk == pytest.approx((0.1, 0.2))
    assert snapshot.flow[0] == pytest.approx((0.5, 0.0))
    assert snapshot.flow[1] == pytest.approx((0.0, 0.5), abs=1.0e-12)


def test_snapshot_rejects_bad_geometry_and_nonfinite_cells():
    with pytest.raises(ValueError, match='cell count'):
        CrowdFieldSnapshot(
            'map', 1.0, 1.0, 1.0, 0.0, 0.0, 2, 1,
            'count_exposure', 1, (cell(),)
        ).validate()

    with pytest.raises(ValueError, match='metric geometry'):
        CrowdFieldSnapshot(
            'map', 1.5, 1.0, 1.0, 0.0, 0.0, 1, 1,
            'count_exposure', 1, (cell(),)
        ).validate()

    invalid = cell()
    invalid = CrowdFieldCell(
        **{**invalid.__dict__, 'density': float('nan')}
    )
    with pytest.raises(ValueError, match='finite'):
        invalid.validate()


def test_message_conversion_is_a_lossless_diagnostic_projection():
    source_cell = cell(4)
    message_cell = SimpleNamespace(**source_cell.__dict__)
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        width_m=1.0,
        height_m=1.0,
        resolution_m=1.0,
        origin_x_m=-1.0,
        origin_y_m=-2.0,
        columns=1,
        rows=1,
        estimator='cusum',
        version=7,
        cells=[message_cell],
    )

    snapshot = CrowdFieldSnapshot.from_message(message)
    assert snapshot.version == 7
    assert snapshot.cells[0] == source_cell
