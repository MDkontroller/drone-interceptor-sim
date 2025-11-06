"""
tracker.py
-----------
Simple Kalman Filter (KF) for smoothing and short-term prediction of
object pixel positions in image space.

The filter tracks the **center of the bounding box (u, v)** in pixels
using a constant-velocity motion model.

This helps stabilize the perception pipeline by:
- Filtering YOLO’s frame-to-frame noise/jitter.
- Allowing short “predict-only” motion during detection dropouts.

Dependencies
------------
Uses the `filterpy` library for KalmanFilter implementation.

State definition
----------------
x = [u, v, du, dv]^T
where:
    u, v  → pixel coordinates of the bounding box center
    du, dv → pixel velocities per frame

Measurement z = [u_meas, v_meas]^T
"""

import numpy as np
from filterpy.kalman import KalmanFilter


class BoxCenterKF:
    """
    Constant-velocity Kalman Filter for pixel center tracking.

    Attributes
    ----------
    kf : filterpy.kalman.KalmanFilter
        The underlying filter instance (4D state, 2D measurement).
    initialized : bool
        Indicates if the filter has received its first measurement.

    Parameters
    ----------
    dt : float, optional
        Time step between frames [seconds]. Default: 1/15 (~15 FPS).
        This controls how strongly the position is propagated between frames.
    """

    def __init__(self, dt: float = 1 / 15.0):
        # Create a 4x4 state model and 2D measurement model
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # State transition matrix (F): constant-velocity model
        # [u, v, du, dv] -> next state prediction
        self.kf.F = np.array([
            [1, 0, dt, 0],   # u_next = u + du*dt
            [0, 1, 0, dt],   # v_next = v + dv*dt
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # Measurement matrix (H): we only observe position (u, v)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Covariance initialization
        self.kf.P *= 100.0  # Large initial uncertainty in state
        self.kf.R *= 5.0    # Measurement noise covariance
        q = 0.1             # Process noise scalar (tune if needed)

        # Process noise covariance (Q)
        self.kf.Q = np.array([
            [q, 0, 0, 0],
            [0, q, 0, 0],
            [0, 0, q, 0],
            [0, 0, 0, q],
        ], dtype=float)

        # Internal state flag
        self.initialized = False


    # ---------------------------------------------------------------------
    # Public Methods
    # ---------------------------------------------------------------------
    def update(self, u: float, v: float):
        """
        Update the filter with a new detection measurement.

        Parameters
        ----------
        u, v : float
            Measured pixel coordinates of the target (e.g., from YOLO bbox center).

        Returns
        -------
        np.ndarray
            Smoothed (u, v) pixel coordinates after the update step.

        Notes
        -----
        - If the filter is not yet initialized, this initializes it
          with the first measurement and zero velocity.
        - After initialization, each call performs:
            predict() → update(z)
          using the measured pixel coordinates.
        """
        z = np.array([u, v], dtype=float)
        if not self.initialized:
            # Initialize state: [u, v, du, dv] with zero initial velocity
            self.kf.x = np.array([u, v, 0.0, 0.0], dtype=float)
            self.initialized = True
        else:
            self.kf.predict()
            self.kf.update(z)
        return self.kf.x[:2].copy()  # Return filtered pixel center


    def predict_only(self):
        """
        Perform a prediction step without a new measurement.

        Useful when the detector temporarily loses the target.

        Returns
        -------
        np.ndarray or None
            Predicted (u, v) pixel coordinates if initialized,
            otherwise None if the filter has not started yet.
        """
        if not self.initialized:
            return None
        self.kf.predict()
        return self.kf.x[:2].copy()


