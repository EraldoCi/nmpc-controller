from typing import NamedTuple

class TrajectoryInput(NamedTuple):
    """Trajectory Function input.

    Attributes:
        Rx: Robot X position state.
        Ry: Robot Y position state.
        Rt: Robot Theta state.
        trajX: X previous position.
        trajY: Y previous position.
        trajTeta: Theta previous reference.
        V: previous linear velocity reference.
        W: previous angular velocity reference.
        N2: prediction horizon end (?).
        trajXp: X reference position.
        trajYp: Y reference position.
    """
    Rx: float
    Ry: float
    Rt: float
    trajX: float
    trajY: float
    trajTeta: float
    V: float
    W: float
    N2: int
    trajXp: float
    trajYp: float