"""
pn_guidance.py
---------------
Baseline guidance primitives for the interceptor.

Implements a discrete-time, 2.5D Proportional Navigation (PN) controller that
generates **lateral** velocity commands in the world (ENU) frame based on the
**line-of-sight (LOS) rate** to the target. A simple first-order smoother is
provided to tame command jitter.

Key ideas
---------
• PN accelerates **perpendicular** to the current LOS with magnitude:
      a_lat = N * Vc * |λ̇|
  where:
      N   = navigation constant (typically 2–5),
      Vc  = closing speed (here approximated by v_self_mag),
      λ̇   = LOS angular rate (around vertical axis in 2.5D).

• In this prototype we estimate λ̇ via the 3D formula:
      ω = (r × v_rel) / |r|²
  and take its z-component as the horizontal-plane LOS rate.

Assumptions / limitations
-------------------------
• r_world ≈ target_position - self_position (in ENU), v_rel_world is relative
  velocity in ENU. If these are poorly estimated (e.g., video-only), ω ≈ 0 and
  the command can collapse to ~0. Consider a bearing-based PN alternative if
  only pixel bearings are available.
• Vertical guidance is presently fixed to zero (level flight). Extend as needed.
• Output velocities are clamped by config limits and then typically converted
  to NED before MAVLink transmission (handled elsewhere).

Tuning
------
• N (navigation constant): higher → more aggressive lateral response.
• dt: integration step. Should match your control loop/update rate.
• v_self_mag: approximate current air/ground speed. If set too small, a_lat
  shrinks; if too large, can be overly aggressive (still clamped later).
"""

import numpy as np
import config as C
from src.interfaces.frame_utils import clamp  # available if needed


def pn_guidance(
    r_world: np.ndarray,
    v_rel_world: np.ndarray,
    v_self_mag: float,
    N: float = 3.0,
    dt: float = 0.05,
):
    """
    Compute a 2.5D PN-based **velocity** command in ENU.

    Parameters
    ----------
    r_world : np.ndarray, shape (3,)
        Relative position vector (target - self) in meters, ENU frame.
    v_rel_world : np.ndarray, shape (3,)
        Relative velocity (target - self) in m/s, ENU frame.
    v_self_mag : float
        Magnitude of interceptor's own speed [m/s]. Used as Vc proxy in PN.
    N : float, optional
        Navigation constant. Typical values: 2–5. Default 3.0.
    dt : float, optional
        Integration timestep [s] used to convert lateral acceleration to a
        velocity increment. Should mirror your frame period.

    Returns
    -------
    np.ndarray, shape (3,)
        Desired velocity command [vx, vy, vz] in ENU [m/s], clamped to
        (V_MAX, VZ_MAX). vz is currently zero (level flight).

    Notes
    -----
    • LOS rate vector (rad/s):
          ω = (r × v_rel) / |r|²
      We use ω_z as the horizontal-plane LOS rate magnitude.

    • Lateral acceleration direction is perpendicular to LOS in XY:
          dir_xy = Rz(+90°) * r_hat_xy

    • Velocity command is obtained by a simple Euler step:
          v_des_xy = a_lat_xy * dt
    """
    # Normalize relative position to obtain LOS direction
    r_norm = np.linalg.norm(r_world) + 1e-6  # avoid division by zero
    r_hat = r_world / r_norm

    # LOS angular-rate vector (ω) via rigid-body kinematics
    # ω = (r × v_rel) / |r|²
    omega_vec = np.cross(r_world, v_rel_world) / (r_norm ** 2)
    omega_z = omega_vec[2]               # 2.5D: yaw (about +z in ENU)
    omega_mag = abs(omega_z)             # magnitude for PN gain

    # PN lateral acceleration magnitude:
    # a_lat = N * Vc * |λ̇|, here we proxy Vc with v_self_mag
    a_lat = N * max(v_self_mag, 1e-3) * omega_mag

    # Direction: perpendicular to LOS in horizontal plane (rotate +90°)
    # If r_hat_xy ~ [rx, ry], perp-left is [-ry, rx].
    dir_xy = np.array([-r_hat[1], r_hat[0], 0.0])
    dir_norm = np.linalg.norm(dir_xy)
    if dir_norm < 1e-6:
        # LOS aligned with z or numerically ill-conditioned → no lateral dir
        dir_xy = np.array([0.0, 0.0, 0.0])
    else:
        dir_xy /= dir_norm

    # Lateral acceleration vector in ENU
    a_lat_vec = a_lat * dir_xy

    # Discrete integration to obtain a velocity increment (very simple model)
    v_des_xy = a_lat_vec[:2] * dt
    v_des_z  = 0.0  # Placeholder for future vertical guidance (e.g., center pixel v)

    # Clamp by config limits before returning
    vx = np.clip(v_des_xy[0], -C.V_MAX,  C.V_MAX)
    vy = np.clip(v_des_xy[1], -C.V_MAX,  C.V_MAX)
    vz = np.clip(v_des_z,     -C.VZ_MAX, C.VZ_MAX)

    return np.array([vx, vy, vz], dtype=float)


def smooth_cmd(prev: float, target: float, alpha: float):
    """
    First-order low-pass filter on a scalar command.

    v <- v_prev + alpha * (target - v_prev)

    Parameters
    ----------
    prev : float
        Previous (filtered) value.
    target : float
        New desired value.
    alpha : float
        Smoothing factor in (0, 1]. Higher = snappier response.

    Returns
    -------
    float
        Smoothed value.
    """
    return prev + alpha * (target - prev)


def apply_smoothing(prev_cmd: np.ndarray, v_des: np.ndarray):
    """
    Apply first-order smoothing to a 3D velocity command.

    Parameters
    ----------
    prev_cmd : np.ndarray, shape (3,)
        Previous filtered velocity command [m/s] (ENU).
    v_des : np.ndarray, shape (3,)
        Newly computed desired velocity command [m/s] (ENU).

    Returns
    -------
    np.ndarray, shape (3,)
        Smoothed velocity command [vx, vy, vz] (ENU).

    Notes
    -----
    • The smoothing factor ALPHA_V is defined in config.py.
    • Apply clamping after smoothing if you need hard bounds on the filtered
      output too (here we clamp only the instantaneous v_des inside pn_guidance).
    """
    vx = smooth_cmd(prev_cmd[0], v_des[0], C.ALPHA_V)
    vy = smooth_cmd(prev_cmd[1], v_des[1], C.ALPHA_V)
    vz = smooth_cmd(prev_cmd[2], v_des[2], C.ALPHA_V)
    return np.array([vx, vy, vz], dtype=float)


# ---------------------------------------------------------------------------
# Suggested extensions (not implemented here)
# ---------------------------------------------------------------------------
# • Vertical channel: Command vz to center vertical pixel error or altitude error.
# • Saturation & rate limits: Add rate-limiters to avoid abrupt changes.
# • Feedforward: If you know target motion model, add feedforward acceleration.
