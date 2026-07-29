"""ROS-independent incremental crowd-grid model."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PersonSample:
    """One pedestrian state and predicted (x, y, absolute-time) samples."""

    identifier: str
    x: float
    y: float
    velocity_x: float
    velocity_y: float
    confidence: float
    predictions: tuple


@dataclass(frozen=True)
class CrowdGridSnapshot:
    """Immutable grid arrays in row-major order."""

    density: tuple
    risk: tuple
    flow_x: tuple
    flow_y: tuple
    observations: tuple


class CrowdGridModel:
    """Accumulate density, predicted risk, and velocity flow grids."""

    def __init__(
        self,
        width_m,
        height_m,
        resolution_m,
        origin_x_m=0.0,
        origin_y_m=0.0,
        half_life_s=30.0,
        prediction_half_life_s=3.0,
    ):
        values = (
            width_m,
            height_m,
            resolution_m,
            half_life_s,
            prediction_half_life_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError(
                'width, height, resolution, and half-life must be positive'
            )
        if not all(math.isfinite(value) for value in (
            origin_x_m,
            origin_y_m,
        )):
            raise ValueError('grid origin must be finite')
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.resolution_m = float(resolution_m)
        self.origin_x_m = float(origin_x_m)
        self.origin_y_m = float(origin_y_m)
        self.half_life_s = float(half_life_s)
        self.prediction_half_life_s = float(prediction_half_life_s)
        self.columns = int(math.ceil(self.width_m / self.resolution_m))
        self.rows = int(math.ceil(self.height_m / self.resolution_m))
        size = self.columns * self.rows
        self._density = [0.0] * size
        self._risk = [0.0] * size
        self._flow_x = [0.0] * size
        self._flow_y = [0.0] * size
        self._flow_weight = [0.0] * size
        self._observations = [0.0] * size
        self._last_update_s = None

    def _index(self, x, y):
        column = math.floor((x - self.origin_x_m) / self.resolution_m)
        row = math.floor((y - self.origin_y_m) / self.resolution_m)
        if column < 0 or row < 0:
            return None
        if column >= self.columns or row >= self.rows:
            return None
        return row * self.columns + column

    def _decay(self, observed_at_s):
        if self._last_update_s is None:
            self._last_update_s = observed_at_s
            return
        elapsed = max(0.0, observed_at_s - self._last_update_s)
        factor = math.exp(-math.log(2.0) * elapsed / self.half_life_s)
        for values in (
            self._density,
            self._risk,
            self._flow_x,
            self._flow_y,
            self._flow_weight,
        ):
            for index, value in enumerate(values):
                values[index] = value * factor
        self._last_update_s = observed_at_s

    def _add_gaussian(self, values, x, y, amount):
        center_column = math.floor(
            (x - self.origin_x_m) / self.resolution_m
        )
        center_row = math.floor(
            (y - self.origin_y_m) / self.resolution_m
        )
        for row_offset in range(-2, 3):
            for column_offset in range(-2, 3):
                column = center_column + column_offset
                row = center_row + row_offset
                if not 0 <= column < self.columns:
                    continue
                if not 0 <= row < self.rows:
                    continue
                squared = column_offset ** 2 + row_offset ** 2
                weight = math.exp(-0.5 * squared)
                index = row * self.columns + column
                values[index] += amount * weight

    def observe(self, observed_at_s, people):
        """Apply one coherent social observation incrementally."""
        if not math.isfinite(observed_at_s) or observed_at_s < 0.0:
            raise ValueError('observation time must be finite and non-negative')
        self._decay(observed_at_s)
        seen = set()
        for person in people:
            if not person.identifier or person.identifier in seen:
                raise ValueError('pedestrian identifiers must be unique')
            seen.add(person.identifier)
            values = (
                person.x,
                person.y,
                person.velocity_x,
                person.velocity_y,
                person.confidence,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError('pedestrian state must be finite')
            if not 0.0 <= person.confidence <= 1.0:
                raise ValueError('pedestrian confidence must be in [0, 1]')
            index = self._index(person.x, person.y)
            if index is None:
                continue
            confidence = person.confidence
            self._add_gaussian(
                self._density,
                person.x,
                person.y,
                confidence,
            )
            self._observations[index] += 1.0
            self._flow_x[index] += confidence * person.velocity_x
            self._flow_y[index] += confidence * person.velocity_y
            self._flow_weight[index] += confidence
            self._add_gaussian(self._risk, person.x, person.y, confidence)
            previous_prediction_at_s = observed_at_s
            for prediction in person.predictions:
                if len(prediction) != 3:
                    raise ValueError(
                        'predictions must contain X, Y, and absolute time'
                    )
                predicted_x, predicted_y, prediction_at_s = prediction
                if not all(math.isfinite(value) for value in prediction):
                    raise ValueError('predictions must be finite')
                if prediction_at_s <= previous_prediction_at_s:
                    raise ValueError(
                        'prediction times must be strictly increasing'
                    )
                previous_prediction_at_s = prediction_at_s
                horizon_s = prediction_at_s - observed_at_s
                prediction_weight = math.exp(
                    -math.log(2.0)
                    * horizon_s
                    / self.prediction_half_life_s
                )
                self._add_gaussian(
                    self._risk,
                    predicted_x,
                    predicted_y,
                    confidence * prediction_weight,
                )

    def snapshot(self):
        """Return normalized flow and the current learned grids."""
        flow_x = []
        flow_y = []
        for index, weight in enumerate(self._flow_weight):
            if weight > 1.0e-12:
                flow_x.append(self._flow_x[index] / weight)
                flow_y.append(self._flow_y[index] / weight)
            else:
                flow_x.append(0.0)
                flow_y.append(0.0)
        return CrowdGridSnapshot(
            tuple(self._density),
            tuple(self._risk),
            tuple(flow_x),
            tuple(flow_y),
            tuple(self._observations),
        )
