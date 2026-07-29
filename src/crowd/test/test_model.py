import pytest

from semaforr_crowd.model import CrowdGridModel, PersonSample


def person(identifier='p1', predictions=()):
    return PersonSample(
        identifier,
        1.2,
        1.2,
        0.5,
        0.0,
        1.0,
        predictions,
    )


def test_density_flow_and_prediction_risk_are_independent():
    model = CrowdGridModel(5.0, 5.0, 1.0)
    model.observe(1.0, [person(predictions=((3.2, 1.2, 2.0),))])
    snapshot = model.snapshot()

    current = 1 * model.columns + 1
    predicted = 1 * model.columns + 3
    assert snapshot.density[current] > snapshot.density[predicted]
    assert snapshot.risk[predicted] > snapshot.density[predicted]
    assert snapshot.flow_x[current] == pytest.approx(0.5)
    assert snapshot.flow_y[current] == pytest.approx(0.0)


def test_decay_and_validation_are_deterministic():
    model = CrowdGridModel(5.0, 5.0, 1.0, half_life_s=1.0)
    model.observe(1.0, [person()])
    before = max(model.snapshot().density)
    model.observe(2.0, [])
    after = max(model.snapshot().density)

    assert after == pytest.approx(before / 2.0)
    with pytest.raises(ValueError, match='unique'):
        model.observe(3.0, [person(), person()])


def test_predictions_require_future_absolute_timestamps():
    model = CrowdGridModel(5.0, 5.0, 1.0)

    with pytest.raises(ValueError, match='strictly increasing'):
        model.observe(2.0, [person(predictions=((2.2, 1.2, 2.0),))])

    with pytest.raises(ValueError, match='strictly increasing'):
        model.observe(3.0, [person(predictions=(
            (2.2, 1.2, 5.0),
            (2.4, 1.2, 4.0),
        ))])
