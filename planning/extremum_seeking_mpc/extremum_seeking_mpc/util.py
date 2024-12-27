from enum import IntEnum
from dataclasses import dataclass


@dataclass
class Position:
    x: float
    y: float


@dataclass
class Pose:
    pos: Position
    yaw: float


@dataclass
class Velocity:
    linear: float
    angular: float


@dataclass
class MpcParameters:
    seek_gain: float
    seek_amp: float
    curvature_max: float
    curvature_min: float


class Side(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_SIDES = 2
