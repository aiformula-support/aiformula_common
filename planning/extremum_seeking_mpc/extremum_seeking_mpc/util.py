from dataclasses import dataclass
from enum import IntEnum


@dataclass
class Position2d:
    x: float = 0.0
    y: float = 0.0


@dataclass
class Pose:
    pos: Position2d = Position2d()
    yaw: float = 0.0


@dataclass
class Velocity:
    linear: float
    angular: float


@dataclass
class ControllerParameters:
    seek_gain: float
    seek_amp: float
    curvature_max: float
    curvature_min: float
    feedback_gain: float
    sin_period: float


@dataclass
class LowPassFilterParameters:  # LowPassFilter x(k+1) = A*x(k) + B*u(k) , y(k) = C * x(k)
    A: float
    B: float
    C: float


class Side(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_SIDES = 2


class Vector2(IntEnum):
    X = 0
    Y = 1
