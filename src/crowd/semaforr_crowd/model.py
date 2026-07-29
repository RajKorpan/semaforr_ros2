"""ROS-independent validation and projection of learned crowd snapshots."""

from dataclasses import dataclass
import math


FLOW_ANGLES = tuple(index * math.pi / 4.0 for index in range(8))


@dataclass(frozen=True)
class CrowdFieldCell:
    density: float
    learned_encounter_risk: float
    directional_flow: tuple
    visibility_exposures: float
    pedestrian_hits: float
    risk_encounters: float
    risk_experiences: float
    confidence: float

    def validate(self):
        values = (
            self.density,
            self.learned_encounter_risk,
            self.visibility_exposures,
            self.pedestrian_hits,
            self.risk_encounters,
            self.risk_experiences,
            self.confidence,
            *self.directional_flow,
        )
        if len(self.directional_flow) != 8:
            raise ValueError('crowd flow must contain eight direction bins')
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError('crowd field cells must be finite and nonnegative')
        if self.confidence > 1.0:
            raise ValueError('crowd field confidence must be within [0, 1]')

    def flow_vector(self):
        return (
            sum(
                value * math.cos(angle)
                for value, angle in zip(self.directional_flow, FLOW_ANGLES)
            ),
            sum(
                value * math.sin(angle)
                for value, angle in zip(self.directional_flow, FLOW_ANGLES)
            ),
        )


@dataclass(frozen=True)
class CrowdFieldSnapshot:
    frame_id: str
    width_m: float
    height_m: float
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    columns: int
    rows: int
    estimator: str
    version: int
    cells: tuple

    def validate(self):
        dimensions = (
            self.width_m,
            self.height_m,
            self.resolution_m,
            self.origin_x_m,
            self.origin_y_m,
        )
        if not self.frame_id or not self.estimator:
            raise ValueError('crowd field frame and estimator must be named')
        if not all(math.isfinite(value) for value in dimensions):
            raise ValueError('crowd field geometry must be finite')
        if (
            self.width_m <= 0.0
            or self.height_m <= 0.0
            or self.resolution_m <= 0.0
            or self.columns <= 0
            or self.rows <= 0
            or self.version <= 0
        ):
            raise ValueError('crowd field geometry and version must be positive')
        if len(self.cells) != self.columns * self.rows:
            raise ValueError('crowd field cell count does not match geometry')
        if (
            self.columns != math.ceil(self.width_m / self.resolution_m)
            or self.rows != math.ceil(self.height_m / self.resolution_m)
        ):
            raise ValueError(
                'crowd field rows and columns do not match metric geometry'
            )
        for cell in self.cells:
            cell.validate()

    @property
    def density(self):
        return tuple(cell.density for cell in self.cells)

    @property
    def risk(self):
        return tuple(cell.learned_encounter_risk for cell in self.cells)

    @property
    def flow(self):
        return tuple(cell.flow_vector() for cell in self.cells)

    @classmethod
    def from_message(cls, message):
        snapshot = cls(
            frame_id=message.header.frame_id,
            width_m=float(message.width_m),
            height_m=float(message.height_m),
            resolution_m=float(message.resolution_m),
            origin_x_m=float(message.origin_x_m),
            origin_y_m=float(message.origin_y_m),
            columns=int(message.columns),
            rows=int(message.rows),
            estimator=message.estimator,
            version=int(message.version),
            cells=tuple(
                CrowdFieldCell(
                    density=float(cell.density),
                    learned_encounter_risk=float(
                        cell.learned_encounter_risk
                    ),
                    directional_flow=tuple(cell.directional_flow),
                    visibility_exposures=float(cell.visibility_exposures),
                    pedestrian_hits=float(cell.pedestrian_hits),
                    risk_encounters=float(cell.risk_encounters),
                    risk_experiences=float(cell.risk_experiences),
                    confidence=float(cell.confidence),
                )
                for cell in message.cells
            ),
        )
        snapshot.validate()
        return snapshot
