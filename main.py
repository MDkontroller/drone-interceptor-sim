import argparse, time
import cv2
import numpy as np

import config as C
from schemas import Observation, Command
from src.perception.detector import YoloDroneDetector
from src.perception.tracker import BoxCenterKF
from src.perception.geometry import los_and_range_from_bbox
from src.control.pn_guidance import pn_guidance, apply_smoothing
from src.interfaces.frame_utils import clamp, enu_to_ned
from src.interfaces import mavlink_utils as M
from src.utils.draw import draw_bbox, put_hud

def compute_yaw_err_from_rhat(r_hat):
    # angle around z from +x axis (ENU)
    yaw = np.arctan2(r_hat[1], r_hat[0])
    return yaw  # [-pi, pi]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, help="Path to an MP4 or video device index (e.g., 0)")
    ap.add_argument("--show", action="store_true", help="Show visualization window")
    ap.add_argument("--send-mavlink", action="store_true", help="Send velocity commands over MAVLink")
    ap.add_argument("--fps", type=float, default=15.0)
    args = ap.parse_args()

    # Source
    try:
        src_index = int(args.video)
        cap = cv2.VideoCapture(src_index)
    except ValueError:
        cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video source: {args.video}")

    det = YoloDroneDetector()
    kf = BoxCenterKF(dt=1.0/max(1.0, args.fps))

    mav = M.connect() if args.send_mavlink else None
    prev_cmd = np.array([0.0, 0.0, 0.0], dtype=float)

    last_det_t = time.time()
    mode = "PN"
    v_self_mag = 5.0  # assume current speed magnitude (mock)

    dt_frame = 1.0/args.fps
    last_r_world = None

    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        h, w = frame.shape[:2]

        # Detect
        xywh, conf = det.detect(frame)

        has_det = xywh is not None and conf >= C.DET_CONF_THRES
        if has_det:
            u, v, bw, bh = xywh
            u_s, v_s = kf.update(u, v)
            last_det_t = t0
        else:
            # Predict-only for a short window when lost
            kf_pred = kf.predict_only()
            if kf_pred is not None:
                u_s, v_s = kf_pred
                bw, bh = (64, 64)  # fallback bbox size for range
            else:
                u_s, v_s, bw, bh = (w/2, h/2, 64, 64)

        # LOS + range (world) from bbox center and width (approx)
        r_hat_world, rng = los_and_range_from_bbox(u_s, v_s, bw)

        # Build a fake relative position vector in world using r_hat * range
        r_world = r_hat_world * rng

        # Relative velocity: finite diff of r_world
        if last_r_world is None:
            v_rel_world = np.zeros(3)
        else:
            v_rel_world = (r_world - last_r_world) / max(1e-3, dt_frame)
        last_r_world = r_world.copy()

        # Compute yaw err for HUD/debug (angle to target around z)
        yaw_err = compute_yaw_err_from_rhat(r_hat_world)

        obs = Observation(
            r_hat=r_hat_world,
            range_m=float(rng),
            rel_vel=v_rel_world,
            bbox_xywh=np.array([u_s, v_s, bw, bh]),
            conf=float(conf),
            yaw_err=float(yaw_err),
        )

        # Guidance (PN baseline)
        v_des_world = pn_guidance(
            r_world=r_world, v_rel_world=v_rel_world, v_self_mag=v_self_mag, N=3.0, dt=dt_frame
        )
        v_cmd_world = apply_smoothing(prev_cmd, v_des_world)
        prev_cmd = v_cmd_world

        # Convert to NED right before sending (today we just print)
        v_ned = enu_to_ned(v_cmd_world)

        # MAVLink send (optional today)
        if args.send_mavlink:
            M.send_velocity_ned(mav, v_ned[0], v_ned[1], v_ned[2], yaw_rate=0.0)

        # Visualization
        if args.show:
            label = f"{conf:.2f}" if has_det else "pred"
            frame = draw_bbox(frame, (u_s, v_s, bw, bh), (0,255,0) if has_det else (0,200,200), label)
            hud = [
                f"mode={mode}",
                f"conf={conf:.2f}",
                f"range~={rng:.1f} m",
                f"v_cmd_ENU=({v_cmd_world[0]:.2f},{v_cmd_world[1]:.2f},{v_cmd_world[2]:.2f})",
            ]
            frame = put_hud(frame, hud)
            cv2.imshow("interceptor", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        # Maintain approx FPS
        elapsed = time.time() - t0
        sleep_t = dt_frame - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
