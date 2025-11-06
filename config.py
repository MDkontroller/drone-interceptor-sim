"""
config.py
-----------
Centralized configuration/constants for the interceptor demo.

These values are imported across perception, guidance, and interfacing modules.
Keep units consistent and document any change when tuning.

Conventions
- Image frame: 640x480 pixels (u right, v down), pinhole intrinsics below.
- World frame (ENU): +x East, +y North, +z Up.
- PX4 external velocity commands are ultimately sent in LOCAL_NED, so we convert
  from ENU→NED right before MAVLink transmission.

Tuning tips
- If your test video/camera differs from 640x480 or focal length, adjust FX, FY, CX, CY.
- If the target appears larger/smaller than expected, refine TARGET_WIDTH_M.
- To make the vehicle feel more/less “twitchy,” adjust V_MAX/VZ_MAX and ALPHA_V.
- Raise DET_CONF_THRES to reduce false positives; lower it to detect smaller/farther targets.
"""

# -----------------------------
# Camera intrinsics (pinhole)
# -----------------------------
# Assumed image size: 640 (width) x 480 (height) pixels.
# FX, FY: focal lengths in pixels along u (x) and v (y).
# CX, CY: principal point (usually near image center).
# NOTE: If you switch camera resolution, update these or compute rays in normalized coords.
FX = 600.0
FY = 600.0
CX = 320.0
CY = 240.0

# ---------------------------------------------
# Target physical prior (for monocular ranging)
# ---------------------------------------------
# Approximate physical width of the target drone (meters).
# Used in a simple pinhole-based depth proxy: range ≈ f * W_real / w_px.
# This is a rough heuristic; expect bias. Tune per target class/scene.
TARGET_WIDTH_M = 0.5  # [m]

# --------------------------------
# Command limits (PX4-friendly)
# --------------------------------
# Velocity limits (ENU before conversion to NED for MAVLink).
# Keep within your PX4/SITL vehicle’s safe operating envelope.
V_MAX = 8.0        # [m/s] max horizontal speed (applies to x,y components)
VZ_MAX = 3.0       # [m/s] max vertical speed (z component)
YAW_RATE_MAX = 1.0 # [rad/s] ~57 deg/s; used if/when yaw rate is commanded

# --------------------------------
# Command smoothing
# --------------------------------
# First-order low-pass on velocity command: v_cmd <- v_cmd + α*(v_des - v_cmd)
# Higher ALPHA_V → snappier (less smoothing). Lower → smoother but laggy.
ALPHA_V = 0.3

# --------------------------------
# Detector thresholds (YOLO)
# --------------------------------
# DET_CONF_THRES: minimum confidence score to accept a detection.
# DET_IOU_THRES: NMS IoU threshold (suppress overlapping boxes).
# Raise confidence to reduce false positives; lower to keep small/weak detections.
DET_CONF_THRES = 0.35
DET_IOU_THRES  = 0.5

# --------------------------------
# Track-loss handling
# --------------------------------
# LOSS_PREDICT_S: duration (seconds) to trust the tracker’s predict-only mode
#   after losing detector lock before declaring a full loss.
# SEARCH_AFTER_S: time (seconds) after last confident detection to switch
#   to a broader search behavior (not yet implemented; placeholder for RL/baseline).
LOSS_PREDICT_S = 0.5  # [s]
SEARCH_AFTER_S = 2.0  # [s]

