import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from enum import Enum, auto
from matplotlib.animation import FuncAnimation


# =========================
# 1. CONFIG DATA CLASSES
# =========================

@dataclass
class GeometryConfig:
    stadium_center: np.ndarray  # (3,)
    R_protected: float          # protected radius (m)
    z_min: float                # min height of protected volume
    z_max: float                # max height of protected volume
    safety_margin: float        # extra margin around protected radius
    R_acting: float             # acting zone radius (unused here now)
    R_outer: float              # outer engagement radius (unused here)


@dataclass
class TubeConfig:
    p_A: np.ndarray             # start of intruder nominal path
    p_B: np.ndarray             # end (inside stadium)
    tube_radius: float          # visual / geometric tube radius
    max_target_speed: float     # max intruder speed (m/s)


@dataclass
class VehicleConfig:
    max_speed: float            # interceptor max speed (m/s)
    base_position: np.ndarray   # interceptor start
    capture_radius: float       # interception radius (m)


# =========================
# 2. UTILITIES
# =========================

def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-8:
        return np.zeros_like(v)
    return v / n


def orthonormal_basis_from_direction(d_hat: np.ndarray):
    d_hat = normalize(d_hat)
    if abs(d_hat[0]) < 0.9:
        temp = np.array([1.0, 0.0, 0.0])
    else:
        temp = np.array([0.0, 1.0, 0.0])
    u1 = np.cross(d_hat, temp)
    u1 = normalize(u1)
    u2 = np.cross(d_hat, u1)
    u2 = normalize(u2)
    return u1, u2


# =========================
# 3. INTRUDER TRAJECTORY
# =========================

class TubeIntruderModel:
    """
    True intruder motion:
    - Nominal straight line p_A -> p_B
    - Smooth lateral deviations bounded by tube_radius
    """

    def __init__(self, tube_cfg: TubeConfig, dt: float):
        self.cfg = tube_cfg
        self.dt = dt

        self.p_A = tube_cfg.p_A
        self.p_B = tube_cfg.p_B
        self.d_vec = self.p_B - self.p_A
        self.L = float(np.linalg.norm(self.d_vec))
        assert self.L > 1e-6, "p_A and p_B must be different."

        self.d_hat = self.d_vec / self.L
        self.u1, self.u2 = orthonormal_basis_from_direction(self.d_hat)

        self.V_t = 0.8 * tube_cfg.max_target_speed  # chosen cruise speed
        self.s = 0.0

        # lateral motion params
        self.omega1 = 0.2
        self.omega2 = 0.4
        self.phi1 = 0.0
        self.phi2 = 1.0

        self.t = 0.0
        self.p_true = self.p_A.copy()
        self.v_true = self.V_t * self.d_hat.copy()

    def step(self):
        self.t += self.dt
        self.s += self.V_t * self.dt
        if self.s > self.L:
            self.s = self.L

        p_nom = self.p_A + self.d_hat * self.s

        w1 = self.cfg.tube_radius * np.sin(self.omega1 * self.t + self.phi1)
        w2 = 0.5 * self.cfg.tube_radius * np.sin(self.omega2 * self.t + self.phi2)
        w_perp = w1 * self.u1 + w2 * self.u2

        norm_w = np.linalg.norm(w_perp)
        if norm_w > self.cfg.tube_radius:
            w_perp = self.cfg.tube_radius * w_perp / norm_w

        p_prev = self.p_true.copy()
        self.p_true = p_nom + w_perp
        self.v_true = (self.p_true - p_prev) / self.dt

        return self.p_true.copy(), self.v_true.copy(), self.s, p_nom.copy()


# =========================
# 4. KALMAN FILTER
# =========================

class TargetTrackerKF:
    """
    Constant-velocity Kalman filter:
    x = [x, y, z, vx, vy, vz]^T
    z = [x, y, z]^T
    """

    def __init__(self, dt: float, process_sigma: float, meas_sigma: float):
        self.dt = dt
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1000.0

        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i + 3] = dt

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        q = process_sigma ** 2
        self.Q = np.eye(6) * q
        r = meas_sigma ** 2
        self.R = np.eye(3) * r

        self.initialized = False

    def initialize(self, z0: np.ndarray):
        self.x[:3] = z0
        self.x[3:] = 0.0
        self.P = np.diag([10, 10, 10, 100, 100, 100])
        self.initialized = True

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        if not self.initialized:
            if z is not None:
                self.initialize(z)
            return

        self.predict()

        if z is None:
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def get_estimate(self):
        return self.x[:3].copy(), self.x[3:].copy(), self.P.copy()


# =========================
# 5. SAFETY FILTER (GEOFENCE)
# =========================

class SafetyFilter:
    """
    Horizontal geofence:
    - R_safe = R_protected + safety_margin
    - If interceptor inside R_safe and commanded radial velocity inward,
      clamp inward component to zero.
    """

    def __init__(self, geom_cfg: GeometryConfig):
        self.cfg = geom_cfg
        self.R_safe = geom_cfg.R_protected + geom_cfg.safety_margin

    def apply(self, p_i: np.ndarray, v_cmd: np.ndarray, max_speed: float) -> np.ndarray:
        center_xy = self.cfg.stadium_center[:2]
        p_xy = p_i[:2]
        delta_xy = p_xy - center_xy
        r = np.linalg.norm(delta_xy)

        if r > 1e-6:
            e_r_xy = delta_xy / r
        else:
            e_r_xy = np.array([1.0, 0.0])

        v_cmd_xy = v_cmd[:2]
        v_r = float(np.dot(v_cmd_xy, e_r_xy))
        v_t_xy = v_cmd_xy - v_r * e_r_xy

        if r <= self.R_safe and v_r < 0.0:
            v_r_safe = 0.0
        else:
            v_r_safe = v_r

        v_safe_xy = v_r_safe * e_r_xy + v_t_xy
        v_safe = np.array([v_safe_xy[0], v_safe_xy[1], v_cmd[2]])

        speed = np.linalg.norm(v_safe)
        if speed > max_speed:
            v_safe = v_safe / speed * max_speed

        return v_safe


# =========================
# 6. CONTROLLERS
# =========================

class PIDGuidanceController:
    """
    Baseline PID-like controller (relative PD):
      e       = p_hat - p_i
      v_rel   = v_i - v_hat
      v_cmd   = v_hat + Kp * e - Kd * v_rel
    """

    def __init__(self, veh_cfg: VehicleConfig, dt: float):
        self.veh_cfg = veh_cfg
        self.dt = dt
        # Tuned for dt = 0.1, current geometry & speeds
        self.Kp = 1.5   # position gain on relative position
        self.Kd = 0.4   # damping on relative velocity

    def compute_command(self,
                        p_i: np.ndarray,
                        v_i: np.ndarray,
                        p_t_hat: np.ndarray,
                        v_t_hat: np.ndarray) -> np.ndarray:
        # Relative position error
        e = p_t_hat - p_i
        dist = np.linalg.norm(e)
        if dist < 1e-3:
            return np.zeros(3)

        # Relative velocity (interceptor wrt target)
        v_rel = v_i - v_t_hat

        # PD on relative state; v_hat term makes us "track" a moving target
        v_cmd = v_t_hat + self.Kp * e - self.Kd * v_rel

        # Saturate to interceptor max speed
        speed = np.linalg.norm(v_cmd)
        if speed > self.veh_cfg.max_speed:
            v_cmd = v_cmd / speed * self.veh_cfg.max_speed

        return v_cmd


class PNGuidanceController:
    """
    PN-flavoured guidance in 3D for a velocity-limited interceptor.

    Idea:
      - Base: pure pursuit along the LOS at max speed (guarantees closing).
      - Add a small lateral correction derived from classical PN:
            a_pn ∝ N * V_c * (LOS_rate × r_hat)
        and apply this as a velocity perturbation:
            v_cmd = V_max * r_hat + a_pn * dt
    """

    def __init__(self, veh_cfg: VehicleConfig, dt: float):
        self.veh_cfg = veh_cfg
        self.dt = dt
        self.N = 3.0          # navigation constant (PN gain)
        self.a_max = 20.0     # max lateral "acceleration" magnitude [m/s^2]

    def compute_command(self,
                        p_i: np.ndarray,
                        v_i: np.ndarray,
                        p_t_hat: np.ndarray,
                        v_t_hat: np.ndarray) -> np.ndarray:
        r = p_t_hat - p_i
        dist = np.linalg.norm(r)
        if dist < 1e-3:
            return np.zeros(3)

        r_hat = r / dist
        v_rel = v_t_hat - v_i   # relative velocity (target - interceptor)

        # Closing speed (positive if closing)
        Vc = -float(np.dot(v_rel, r_hat))

        # LOS rate vector (3D):
        # lambda_dot_vec = (r x v_rel) / |r|^2
        r_cross_vrel = np.cross(r, v_rel)
        r_norm_sq = max(dist * dist, 1e-6)
        lambda_dot_vec = r_cross_vrel / r_norm_sq

        # PN lateral "acceleration": a_pn = N * Vc * (lambda_dot_vec × r_hat)
        a_pn = self.N * Vc * np.cross(lambda_dot_vec, r_hat)

        # Limit the PN acceleration magnitude
        a_norm = np.linalg.norm(a_pn)
        if a_norm > self.a_max:
            a_pn = a_pn / a_norm * self.a_max

        # Base: pure pursuit along LOS at max speed
        v_pursuit = self.veh_cfg.max_speed * r_hat

        # Apply PN correction as a small velocity delta
        v_cmd = v_pursuit + a_pn * self.dt

        # Final saturation to max speed
        speed = np.linalg.norm(v_cmd)
        if speed > self.veh_cfg.max_speed:
            v_cmd = v_cmd / speed * self.veh_cfg.max_speed

        return v_cmd


# =========================
# 7. SINGLE TRIAL SIMULATION
# =========================

def simulate_single_trial(meas_sigma: float,
                          controller_type: str,
                          seed: int = None):
    """
    Simulate one scenario for a given measurement noise and controller type.

    Returns:
        intercepted (bool),
        intercept_time (float or None),
        stop_reason (str)
    """
    if seed is not None:
        np.random.seed(seed)

    dt = 0.1

    # Geometry & configs
    geom_cfg = GeometryConfig(
        stadium_center=np.array([0.0, 0.0, 0.0]),
        R_protected=100.0,
        z_min=0.0,
        z_max=100.0,
        safety_margin=5.0,
        R_acting=150.0,
        R_outer=250.0,
    )

    tube_cfg = TubeConfig(
        p_A=np.array([-500.0, 0.0, 50.0]),
        p_B=np.array([0.0, 0.0, 50.0]),
        tube_radius=20.0,
        max_target_speed=25.0,
    )

    veh_cfg = VehicleConfig(
        max_speed=40.0,
        base_position=np.array([0.0, -300.0, 50.0]),
        capture_radius=2.0,
    )

    intruder = TubeIntruderModel(tube_cfg, dt)
    tracker = TargetTrackerKF(dt=dt, process_sigma=1.0, meas_sigma=meas_sigma)
    safety_filter = SafetyFilter(geom_cfg)

    # Select controller
    if controller_type == "pid":
        controller = PIDGuidanceController(veh_cfg, dt)
    elif controller_type == "pn":
        controller = PNGuidanceController(veh_cfg, dt)
    else:
        raise ValueError(f"Unknown controller_type: {controller_type}")

    p_i = veh_cfg.base_position.copy()
    v_i = np.zeros(3)

    intercepted = False
    intercept_time = None
    stop_reason = None

    max_steps = 2000
    for step in range(max_steps):
        t = step * dt

        # --- True intruder motion ---
        p_true, v_true, s, p_nom = intruder.step()

        # --- Measurement (noisy, dropout) ---
        if np.random.rand() < 0.95:
            z = p_true + np.random.randn(3) * meas_sigma
        else:
            z = None

        tracker.update(z)

        if tracker.initialized:
            p_hat, v_hat, P = tracker.get_estimate()
        else:
            # fallback before first measurement
            p_hat, v_hat = p_true.copy(), v_true.copy()

        # --- Guidance + safety ---
        v_cmd = controller.compute_command(p_i, v_i, p_hat, v_hat)
        v_safe = safety_filter.apply(p_i, v_cmd, veh_cfg.max_speed)

        p_i = p_i + v_safe * dt
        v_i = v_safe

        # --- Distances & checks ---
        dist_intr = np.linalg.norm(p_i - p_true)

        # Interception success
        if dist_intr <= veh_cfg.capture_radius and not intercepted:
            intercepted = True
            intercept_time = t
            stop_reason = "intercept"
            break

        # Intruder hits protected zone -> failure
        r_true = np.linalg.norm(p_true[:2] - geom_cfg.stadium_center[:2])
        if r_true <= geom_cfg.R_protected:
            stop_reason = "intruder_hit_protected"
            break

    if stop_reason is None:
        stop_reason = "max_steps_reached"

    return intercepted, intercept_time, stop_reason


# =========================
# 8. MONTE CARLO BENCHMARK (PID vs PN)
# =========================

def run_monte_carlo_benchmark():
    NOISE_LEVELS = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0,
                    12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
                    50, 100, 150]
    N_TRIALS = 50

    success_pid = []
    success_pn  = []

    tmean_pid = []
    tmean_pn  = []

    print("=== Monte Carlo Benchmark (PID vs PN) ===")
    print(f"Trials per sigma: {N_TRIALS}")
    print("Noise levels (sigma):", NOISE_LEVELS)
    print()

    for idx_sigma, sigma in enumerate(NOISE_LEVELS):
        pid_successes = 0
        pn_successes  = 0

        pid_times = []
        pn_times  = []

        for trial in range(N_TRIALS):
            # Use SAME seed for both controllers at this (sigma, trial)
            seed = 1000 * idx_sigma + trial

            # PID controller
            intercepted_p, t_p, reason_p = simulate_single_trial(
                meas_sigma=sigma,
                controller_type="pid",
                seed=seed,
            )
            if intercepted_p:
                pid_successes += 1
                pid_times.append(t_p)

            # PN-flavoured controller
            intercepted_n, t_n, reason_n = simulate_single_trial(
                meas_sigma=sigma,
                controller_type="pn",
                seed=seed,
            )
            if intercepted_n:
                pn_successes += 1
                pn_times.append(t_n)

        # Success rates
        rate_pid = 100.0 * pid_successes / N_TRIALS
        rate_pn  = 100.0 * pn_successes  / N_TRIALS

        success_pid.append(rate_pid)
        success_pn.append(rate_pn)

        # Mean intercept times over successful trials
        mean_t_pid = float(np.mean(pid_times)) if pid_times else np.nan
        mean_t_pn  = float(np.mean(pn_times))  if pn_times  else np.nan

        tmean_pid.append(mean_t_pid)
        tmean_pn.append(mean_t_pn)

        print(f"Sigma = {sigma:.1f} m:")
        print(f"  PID baseline : success = {rate_pid:5.1f}% "
              f"(mean t_int = {mean_t_pid:6.2f} s)")
        print(f"  PN-flavoured : success = {rate_pn:5.1f}% "
              f"(mean t_int = {mean_t_pn:6.2f} s)")
        print()

    # =========================
    # Plot success rate vs noise
    # =========================
    plt.figure(figsize=(8, 5))
    plt.plot(NOISE_LEVELS, success_pid, marker="s",
             label="PID baseline (relative PD)")
    plt.plot(NOISE_LEVELS, success_pn, marker="x",
             label="PN-flavoured (pursuit + PN)")

    plt.xlabel("Measurement noise σ [m]")
    plt.ylabel("Success rate [%]")
    plt.title("Interception Success Rate vs Measurement Noise (PID vs PN)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.show()



from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def animate_3d_trajectories(
    geom_cfg: GeometryConfig,
    tube_radius: float,
    intr_true_positions,
    intr_est_positions,
    interceptor_positions,
    intercept_index,
    capture_radius: float,
    dt: float,
    title: str,
):
    intr_true = np.array(intr_true_positions)
    intr_est = np.array(intr_est_positions)
    intc = np.array(interceptor_positions)
    N = intr_true.shape[0]

    if intercept_index is not None:
        intr_true = intr_true[:intercept_index + 1]
        intr_est = intr_est[:intercept_index + 1]
        intc = intc[:intercept_index + 1]
        N = intr_true.shape[0]

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection="3d")

    center = geom_cfg.stadium_center

    # Stadium protected cylinder
    theta = np.linspace(0, 2*np.pi, 200)
    R = geom_cfg.R_protected
    x_circ = center[0] + R*np.cos(theta)
    y_circ = center[1] + R*np.sin(theta)
    z_bottom = np.full_like(theta, geom_cfg.z_min)
    z_top = np.full_like(theta, geom_cfg.z_max)
    ax.plot(x_circ, y_circ, z_bottom, "r-", label="Protected volume")
    ax.plot(x_circ, y_circ, z_top, "r-")

    # Acting zone
    R_act = geom_cfg.R_acting
    x_act = center[0] + R_act*np.cos(theta)
    y_act = center[1] + R_act*np.sin(theta)
    z_act = np.full_like(theta, geom_cfg.z_min)
    ax.plot(x_act, y_act, z_act, "y--", label="Acting zone")

    # Dummy proxy for tube legend
    ax.plot([], [], [], color="gray", linestyle="-", label="Estimated tube")

    # Trajectory lines and points
    intr_true_line, = ax.plot([], [], [], "b-", linewidth=2, label="Intruder (true)")
    intr_est_line,  = ax.plot([], [], [], "c--", linewidth=1.5, label="Intruder (estimated)")
    intc_line,      = ax.plot([], [], [], "g-", linewidth=2, label="Interceptor")

    intr_true_point, = ax.plot([], [], [], "bo", markersize=5)
    intr_est_point,  = ax.plot([], [], [], "co", markersize=5)
    intc_point,      = ax.plot([], [], [], "go", markersize=5)

    # Tube segments (wireframes)
    tube_segments = []

    # Intercept point & capture sphere (if any)
    intercept_point = None
    if intercept_index is not None:
        intercept_point = intr_true[-1]
        ax.scatter(intercept_point[0], intercept_point[1], intercept_point[2],
                   c="k", marker="*", s=120, label="Intercept point")

        u = np.linspace(0, 2*np.pi, 24)
        v = np.linspace(0, np.pi, 12)
        xs = intercept_point[0] + capture_radius*np.outer(np.cos(u), np.sin(v))
        ys = intercept_point[1] + capture_radius*np.outer(np.sin(u), np.sin(v))
        zs = intercept_point[2] + capture_radius*np.outer(np.ones_like(u), np.cos(v))
        ax.plot_wireframe(xs, ys, zs, linewidth=0.4, alpha=0.4, color="k",
                          label="Capture sphere")

    # Axis limits
    all_xyz = np.vstack((intr_true, intc))
    mins = np.min(all_xyz, axis=0) - 50.0
    maxs = np.max(all_xyz, axis=0) + 50.0
    max_range = np.max(maxs - mins)
    mid = 0.5 * (maxs + mins)
    ax.set_xlim(mid[0] - max_range/2, mid[0] + max_range/2)
    ax.set_ylim(mid[1] - max_range/2, mid[1] + max_range/2)
    ax.set_zlim(mid[2] - max_range/2, mid[2] + max_range/2)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    ax.legend(loc="upper left")

    def draw_tube_segment(P0, P1, radius):
        d = P1 - P0
        L = np.linalg.norm(d)
        if L < 1e-3:
            return None
        d_hat = d / L
        u1, u2 = orthonormal_basis_from_direction(d_hat)

        phi = np.linspace(0, 2*np.pi, 18)
        zeta = np.linspace(0, L, 6)
        Phi, Zeta = np.meshgrid(phi, zeta)

        X = (P0[0]
             + Zeta*d_hat[0]
             + radius*np.cos(Phi)*u1[0]
             + radius*np.sin(Phi)*u2[0])
        Y = (P0[1]
             + Zeta*d_hat[1]
             + radius*np.cos(Phi)*u1[1]
             + radius*np.sin(Phi)*u2[1])
        Z = (P0[2]
             + Zeta*d_hat[2]
             + radius*np.cos(Phi)*u1[2]
             + radius*np.sin(Phi)*u2[2])

        return ax.plot_wireframe(X, Y, Z, linewidth=0.5, color="gray", alpha=0.3)

    def update(frame):
        # Paths
        intr_true_line.set_data(intr_true[:frame+1, 0], intr_true[:frame+1, 1])
        intr_true_line.set_3d_properties(intr_true[:frame+1, 2])

        intr_est_line.set_data(intr_est[:frame+1, 0], intr_est[:frame+1, 1])
        intr_est_line.set_3d_properties(intr_est[:frame+1, 2])

        intc_line.set_data(intc[:frame+1, 0], intc[:frame+1, 1])
        intc_line.set_3d_properties(intc[:frame+1, 2])

        # Points
        intr_true_point.set_data(intr_true[frame, 0:1], intr_true[frame, 1:2])
        intr_true_point.set_3d_properties(intr_true[frame, 2:3])

        intr_est_point.set_data(intr_est[frame, 0:1], intr_est[frame, 1:2])
        intr_est_point.set_3d_properties(intr_est[frame, 2:3])

        intc_point.set_data(intc[frame, 0:1], intc[frame, 1:2])
        intc_point.set_3d_properties(intc[frame, 2:3])

        # Tube segment
        if frame > 0:
            seg = draw_tube_segment(intr_est[frame-1], intr_est[frame], tube_radius)
            if seg is not None:
                tube_segments.append(seg)

        return (intr_true_line, intr_est_line, intc_line,
                intr_true_point, intr_est_point, intc_point)

    interval_ms = int(dt * 1000)
    anim = FuncAnimation(
        fig,
        update,
        frames=N,
        interval=interval_ms,
        blit=False,
    )

    plt.tight_layout()
    plt.show()


def run_single_demo(controller_type: str,
                    meas_sigma: float,
                    seed: int = None):
    """
    Run ONE visual demo for a given controller_type ('pn' or 'pid')
    and measurement noise sigma, using the SAME geometry and
    parameters as in the Monte Carlo benchmark.
    """
    if seed is not None:
        np.random.seed(seed)

    dt = 0.1

    # Geometry & configs (same as simulate_single_trial!)
    geom_cfg = GeometryConfig(
        stadium_center=np.array([0.0, 0.0, 0.0]),
        R_protected=100.0,
        z_min=0.0,
        z_max=100.0,
        safety_margin=5.0,
        R_acting=150.0,
        R_outer=250.0,
    )

    tube_cfg = TubeConfig(
        p_A=np.array([-500.0, 0.0, 50.0]),
        p_B=np.array([0.0, 0.0, 50.0]),
        tube_radius=20.0,
        max_target_speed=25.0,
    )

    veh_cfg = VehicleConfig(
        max_speed=40.0,
        base_position=np.array([0.0, -300.0, 50.0]),
        capture_radius=2.0,   # if you want strict consistency with simulate_single_trial
    )

    intruder = TubeIntruderModel(tube_cfg, dt)
    tracker = TargetTrackerKF(dt=dt, process_sigma=1.0, meas_sigma=meas_sigma)
    safety_filter = SafetyFilter(geom_cfg)

    # Pick controller
    if controller_type == "pid":
        controller = PIDGuidanceController(veh_cfg, dt)
        title_prefix = "PID"
    elif controller_type == "pn":
        controller = PNGuidanceController(veh_cfg, dt)
        title_prefix = "PN"
    else:
        raise ValueError(f"Unknown controller_type: {controller_type}")

    # State
    p_i = veh_cfg.base_position.copy()
    v_i = np.zeros(3)

    intercepted = False
    intercept_time = None
    intercept_index = None

    intr_true_positions = []
    intr_est_positions = []
    interceptor_positions = []

    max_steps = 2000
    for step in range(max_steps):
        t = step * dt

        # --- True intruder motion ---
        p_true, v_true, s, p_nom = intruder.step()
        intr_true_positions.append(p_true.copy())

        # --- Measurement (noisy, dropout) ---
        if np.random.rand() < 0.95:
            z = p_true + np.random.randn(3) * meas_sigma
        else:
            z = None

        tracker.update(z)

        if tracker.initialized:
            p_hat, v_hat, P = tracker.get_estimate()
        else:
            p_hat, v_hat = p_true.copy(), v_true.copy()

        intr_est_positions.append(p_hat.copy())

        # --- Guidance + safety ---
        v_cmd = controller.compute_command(p_i, v_i, p_hat, v_hat)
        v_safe = safety_filter.apply(p_i, v_cmd, veh_cfg.max_speed)

        p_i = p_i + v_safe * dt
        v_i = v_safe
        interceptor_positions.append(p_i.copy())

        # --- Distances & checks ---
        dist_intr = np.linalg.norm(p_i - p_true)

        if dist_intr <= veh_cfg.capture_radius and not intercepted:
            intercepted = True
            intercept_time = t
            intercept_index = step
            break

        r_true = np.linalg.norm(p_true[:2] - geom_cfg.stadium_center[:2])
        if r_true <= geom_cfg.R_protected:
            # Intruder hit protected zone -> failure
            break

    title = f"{title_prefix} controller, σ = {meas_sigma:.1f} m"
    animate_3d_trajectories(
        geom_cfg=geom_cfg,
        tube_radius=tube_cfg.tube_radius,
        intr_true_positions=intr_true_positions,
        intr_est_positions=intr_est_positions,
        interceptor_positions=interceptor_positions,
        intercept_index=intercept_index,
        capture_radius=veh_cfg.capture_radius,
        dt=dt,
        title=title,
    )





#if __name__ == "__main__":
#    print("Starting")
#    run_monte_carlo_benchmark()


if __name__ == "__main__":
    print("Starting PID vs PN visual + Monte Carlo benchmark")

    # Noise levels for *visualization* and for the Monte Carlo benchmark
    NOISE_LEVELS = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 100.0]  # you can also reuse full MC list
    NOISE_LEVELS = [0.0, 10.0, 30.0]  # you can also reuse full MC list


    # For each noise level: show PN demo, then PID demo
    for sigma in NOISE_LEVELS:
        print(f"\n=== σ = {sigma} m: PN demo ===")
        # Same seed so PN and PID see same noise realization at this sigma
        run_single_demo(controller_type="pn", meas_sigma=sigma, seed=42)

        print(f"\n=== σ = {sigma} m: PID demo ===")
        run_single_demo(controller_type="pid", meas_sigma=sigma, seed=42)

    # After all visualizations, run the Monte Carlo benchmark
    print("\n=== Running full Monte Carlo benchmark (PID vs PN) ===")
    run_monte_carlo_benchmark()
    
