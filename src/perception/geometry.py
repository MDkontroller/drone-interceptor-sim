"""
geometry.py
------------
Geometric utility functions for estimating line-of-sight (LOS) direction
and approximate range to a detected target, based on 2D image geometry.

Used by the perception → control interface to build a 3D observation
vector from a single bounding box in the camera frame.

In this prototype, the methods make simplifying assumptions:
- The camera optical axis is aligned with the world frame (no rotation).
- Monocular depth is estimated only from bounding-box width.
These approximations are fine for mock demos and early tests, but
they should be replaced by proper camera extrinsics or multi-view
geometry once the simulation is connected.
"""

import numpy as np
import config as C
from src.interfaces.frame_utils import pixel_to_camera_ray


# ---------------------------------------------------------------------------
# Monocular range estimation
# ---------------------------------------------------------------------------
def estimate_range_from_bbox_width_px(bbox_w_px: float):
    """
    Roughly estimate target distance from its apparent width in pixels.

    Parameters
    ----------
    bbox_w_px : float
        Detected bounding-box width in pixels.

    Returns
    -------
    float
        Estimated distance [meters] to the target.

    Notes
    -----
    This uses a pinhole-camera approximation:

        range ≈ f * W_real / w_px

    where:
        f       = focal length in pixels (≈ C.FX)
        W_real  = real-world target width (≈ 0.5 m for small drones)
        w_px    = measured bounding-box width in pixels

    Because this ignores orientation and perspective distortion,
    it’s mainly useful for relative or qualitative distance cues.
    """
    w = max(1.0, bbox_w_px)  # prevent divide-by-zero
    return (C.FX * C.TARGET_WIDTH_M) / w


# ---------------------------------------------------------------------------
# LOS vector and range estimation
# ---------------------------------------------------------------------------
def los_and_range_from_bbox(u: float, v: float, bbox_w_px: float):
    """
    Compute the line-of-sight unit vector and approximate range
    from the image pixel coordinates of a detection.

    Parameters
    ----------
    u, v : float
        Pixel coordinates of the target’s bounding-box center.
        (u right, v down, origin top-left)

    bbox_w_px : float
        Width of the detected bounding box [pixels].

    Returns
    -------
    (r_hat_world, range_m) : (np.ndarray(3,), float)
        r_hat_world : Unit LOS direction vector (x,y,z) in world coordinates.
        range_m     : Estimated range [m].

    Assumptions
    -----------
    - The camera frame is aligned with the world frame
      (i.e., no camera-to-body or body-to-world rotation applied).
    - Later, when integrated with PX4 or Isaac Sim, you’ll replace this
      with proper camera → body → world transformations.

    Example
    -------
    >>> r_hat, rng = los_and_range_from_bbox(320, 240, 80)
    >>> print(r_hat, rng)
    [0.0, 0.0, 1.0]  3.75
    """
    # Compute normalized ray in camera coordinates
    ray_cam = pixel_to_camera_ray(u, v)

    # Normalize to unit length → LOS direction
    r_hat_world = ray_cam / np.linalg.norm(ray_cam)

    # Estimate approximate range (monocular)
    rng = estimate_range_from_bbox_width_px(bbox_w_px)

    return r_hat_world, rng
