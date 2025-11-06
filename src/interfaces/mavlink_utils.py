"""
Stub for now. When sim is ready, set MAVLINK_URL to something like 'udpout:127.0.0.1:14540'
and call send_velocity_ned(...) inside your main loop.
"""
from typing import Optional
try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

MAVLINK_URL = None  # set to 'udpout:127.0.0.1:14540' when ready

def connect():
    if mavutil is None or MAVLINK_URL is None:
        return None
    return mavutil.mavlink_connection(MAVLINK_URL)

def send_velocity_ned(conn, vx, vy, vz, yaw_rate):
    if conn is None:
        return  # stub today
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # ignore pos/acc fields; enable velocity + yaw_rate
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        yaw_rate, 0
    )
