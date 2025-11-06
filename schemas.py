"""
schemas.py
-----------
Lightweight data containers (using Python dataclasses) for key data exchanged
between perception, decision, and control modules.

These define the standardized structure of information that flows through
the pipeline — mainly:
- `Observation`: what the perception system "sees" at a given frame.
- `Command`: what the control system "decides" to send (desired motion).

This helps maintain clean interfaces and avoids hard-to-track dicts/arrays.
"""

from dataclasses import dataclass
import numpy as np


# ---------------------------------------------------------------------------
# Observation
# ---------------------------------------------------------------------------
@dataclass
class Observation:
    """
    Represents a single perception snapshot passed to the controller.

    Attributes
    ----------
    r_hat : np.ndarray (3,)
        Unit line-of-sight (LOS) vector in world or body coordinates.
        - For now, assumed to be in the world frame (ENU).
        - Derived from pixel coordinates via camera intrinsics.

    range_m : float
        Estimated distance to the target [meters].
        - Computed using a rough monocular estimate from the bounding box width.

    rel_vel : np.ndarray (3,)
        Relative velocity vector between target and interceptor [m/s].
        - In the current prototype, estimated from finite differences of LOS.

    bbox_xywh : np.ndarray (4,)
        Bounding box in pixel coordinates (center_x, center_y, width, height).
        - Used for visualization and as raw observation data.

    conf : float
        Confidence score of the detector (0–1).
        - Used to gate detections (below threshold → predictive tracking).

    yaw_err : float
        Yaw error angle [radians] between the interceptor’s current heading
        (assumed x-axis in ENU) and the line-of-sight to the target.
        - Positive when target is to the left, negative to the right.
    """
    r_hat: np.ndarray
    range_m: float
    rel_vel: np.ndarray
    bbox_xywh: np.ndarray
    conf: float
    yaw_err: float


# ---------------------------------------------------------------------------
# Command
# ---------------------------------------------------------------------------
@dataclass
class Command:
    """
    Represents a single velocity command to be sent to the autopilot.

    Attributes
    ----------
    vx, vy, vz : float
        Desired linear velocity components [m/s] in the ENU frame.
        - These are later converted to NED before sending via MAVLink.

    yaw_rate : float
        Desired yaw rate [rad/s].
        - Positive = counterclockwise turn (around +z in ENU).
    """
    vx: float
    vy: float
    vz: float
    yaw_rate: float
