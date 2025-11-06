import numpy as np
import config as C

def pixel_to_camera_ray(u: float, v: float):
    """Return unit ray in camera frame from pixel coordinates using pinhole intrinsics."""
    x = (u - C.CX) / C.FX
    y = (v - C.CY) / C.FY
    z = 1.0
    vec = np.array([x, y, z], dtype=float)
    return vec / np.linalg.norm(vec)

def enu_to_ned(vec_enu: np.ndarray):
    """Convert ENU -> NED. ENU=(+E,+N,+U) to NED=(+N,+E,-D)."""
    e, n, u = vec_enu
    return np.array([n, e, -u], dtype=float)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))
