# ==============================================================================
# Pegasus Nonlinear Flight Script (test.py)
#
# This script uses a custom, pure Python control backend (NonlinearController)
# to make the drone follow a hard-coded, aggressive trajectory, bypassing PX4.
# ==============================================================================

# STEP 1: Initialize SimulationApp FIRST
from isaacsim import SimulationApp
# Note: "headless": False is required to display the Isaac Sim GUI
simulation_app = SimulationApp({"headless": False})

# STEP 2: NOW import everything else
import carb
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
import numpy as np

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends import Backend # Import the base Backend class
from pegasus.simulator.logic.state import State
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS


# ==============================================================================
# CUSTOM CONTROLLER LOGIC (Implemented as a class within this file)
# ==============================================================================

class NonlinearController(Backend):
    """
    A nonlinear controller class based on the Pegasus example. 
    It tracks a hard-coded exponential trajectory using pure Python.
    """

    def __init__(self, 
        trajectory_file: str = None, # We will ignore this and use the built-in trajectory
        results_file: str=None, 
        reverse=False, 
        Kp=[10.0, 10.0, 10.0],
        Kd=[8.5, 8.5, 8.5],
        Ki=[1.50, 1.50, 1.50],
        Kr=[3.5, 3.5, 3.5],
        Kw=[0.5, 0.5, 0.5]):
        
        # The current rotor references [rad/s]
        self.input_ref = [0.0, 0.0, 0.0, 0.0]
        self.p = np.zeros((3,))                   
        self.R: Rotation = Rotation.identity()    
        self.w = np.zeros((3,))                   
        self.v = np.zeros((3,))                   
        self.a = np.zeros((3,))                   
        
        # Control gains
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)
        self.Ki = np.diag(Ki)
        self.Kr = np.diag(Kr)
        self.Kw = np.diag(Kw)
        self.int = np.array([0.0, 0.0, 0.0])
        
        # Dynamic parameters
        self.m = 1.50        # Mass in Kg
        self.g = 9.81       # The gravity acceleration ms^-2
        
        # Trajectory setup (using built-in trajectory since trajectory_file is None)
        self.trajectory = None
        self.index = 0
        self.max_index = 1
        self.total_time = -5.0 # Start parametric time at -5.0
        self.reverse = reverse
        self.reveived_first_state = False
        
        # Statistics (simplified for this self-contained example)
        self.results_files = results_file
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_error_over_time = []

    def start(self):
        self.reset_statistics()

    def stop(self):
        # We skip saving statistics for simplicity in this single-file runnable example
        pass

    def update_sensor(self, sensor_type: str, data):
        # We use the state directly, ignoring sensor data for this controller
        pass

    def update_state(self, state: State):
        """Method that updates the current state of the vehicle."""
        self.p = state.position
        self.R = Rotation.from_quat(state.attitude)
        self.w = state.angular_velocity
        self.v = state.linear_velocity
        self.reveived_first_state = True

    def input_reference(self):
        """Returns the target angular velocities for each rotor."""
        return self.input_ref

    def update(self, dt: float):
        """Implements the nonlinear control law and updates rotor references."""
        
        if self.reveived_first_state == False:
            return
        
        # --- Update the references for the controller to track ---
        self.total_time += dt
        
        # Use built-in exponential trajectory (t is the parametric value)
        s = 0.6
        p_ref = self.pd(self.total_time, s, self.reverse)
        v_ref = self.d_pd(self.total_time, s, self.reverse)
        a_ref = self.dd_pd(self.total_time, s, self.reverse)
        j_ref = self.ddd_pd(self.total_time, s, self.reverse)
        yaw_ref = self.yaw_d(self.total_time, s)
        yaw_rate_ref = self.d_yaw_d(self.total_time, s)
        
        # --- Start the controller implementation ---
        # Compute the tracking errors
        ep = self.p - p_ref
        ev = self.v - v_ref
        self.int = self.int +  (ep * dt)
        ei = self.int
        
        # Compute F_des term
        F_des = -(self.Kp @ ep) - (self.Kd @ ev) - (self.Ki @ ei) + np.array([0.0, 0.0, self.m * self.g]) + (self.m * a_ref)
        
        # Get the current axis Z_B
        Z_B = self.R.as_matrix()[:,2]
        
        # Get the desired total thrust in Z_B direction (u_1)
        u_1 = F_des @ Z_B
        
        # Compute the desired body-frame axis Z_b
        Z_b_des = F_des / np.linalg.norm(F_des)
        
        # Compute X_C_des 
        X_c_des = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])
        
        # Compute Y_b_des
        Z_b_cross_X_c = np.cross(Z_b_des, X_c_des)
        Y_b_des = Z_b_cross_X_c / np.linalg.norm(Z_b_cross_X_c)
        
        # Compute X_b_des
        X_b_des = np.cross(Y_b_des, Z_b_des)
        
        # Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]
        R = self.R.as_matrix()
        
        # Compute the rotation error
        e_R = 0.5 * self.vee((R_des.T @ R) - (R.T @ R_des))
        
        # Compute an approximation of the current vehicle acceleration in the inertial frame
        self.a = (u_1 * Z_B) / self.m - np.array([0.0, 0.0, self.g])
        
        # Compute the desired angular velocity
        hw = (self.m / u_1) * (j_ref - np.dot(Z_b_des, j_ref) * Z_b_des) 
        w_des = np.array([-np.dot(hw, Y_b_des), 
                           np.dot(hw, X_b_des), 
                           yaw_rate_ref * Z_b_des[2]])
                           
        # Compute the angular velocity error
        e_w = self.w - w_des
        
        # Compute the torques to apply on the rigid body
        tau = -(self.Kr @ e_R) - (self.Kw @ e_w)
        
        # Use the allocation matrix provided by the Multirotor vehicle to convert
        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)

        # Statistics logging (minimal)
        self.time_vector.append(self.total_time)
        self.desired_position_over_time.append(p_ref)
        self.position_error_over_time.append(ep)

    @staticmethod
    def vee(S):
        """Auxiliary function that computes the 'v' map which takes elements from so(3) to R^3."""
        return np.array([-S[1,2], S[0,2], -S[0,1]])
    
    def reset_statistics(self):
        self.index = 0
        self.total_time = -5.0
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_error_over_time = []
        
    # --- Exponential Trajectory Definition (Hard-coded) ---
    def pd(self, t, s, reverse=False):
        """Desired position."""
        x = t
        z = 1 / s * np.exp(-0.5 * np.power(t/s, 2)) + 1.0
        y = 1 / s * np.exp(-0.5 * np.power(t/s, 2))
        if reverse == True:
            y = -1 / s * np.exp(-0.5 * np.power(t/s, 2)) + 4.5
        return np.array([x,y,z])
        
    def d_pd(self, t, s, reverse=False):
        """Desired velocity."""
        x = 1.0
        y = -(t * np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,3)
        z = -(t * np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,3)
        if reverse == True:
            y = (t * np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,3)
        return np.array([x,y,z])
        
    def dd_pd(self, t, s, reverse=False):
        """Desired acceleration."""
        x = 0.0
        y = (np.power(t,2)*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,5) - np.exp(-np.power(t,2)/(2*np.power(s,2)))/np.power(s,3)
        z = (np.power(t,2)*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,5) - np.exp(-np.power(t,2)/(2*np.power(s,2)))/np.power(s,3)
        if reverse == True:
            y = np.exp(-np.power(t,2)/(2*np.power(s,2)))/np.power(s,3) - (np.power(t,2)*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,5)
        return np.array([x,y,z])
        
    def ddd_pd(self, t, s, reverse=False):
        """Desired jerk."""
        x = 0.0
        y = (3*t*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,5) - (np.power(t,3)*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,7)
        z = (3*t*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,5) - (np.power(t,3)*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,7)
        if reverse == True:
            y = (np.power(t,3)*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,7) - (3*t*np.exp(-np.power(t,2)/(2*np.power(s,2))))/np.power(s,5)
        return np.array([x,y,z])
        
    def yaw_d(self, t, s):
        """Desired yaw."""
        return 0.0
    
    def d_yaw_d(self, t, s):
        """Desired yaw rate."""
        return 0.0
    
    def reset(self):
        pass
        
    def update_graphical_sensor(self, sensor_type: str, data):
        pass

# ==============================================================================
# MAIN SIMULATION SCRIPT
# ==============================================================================

class PegasusApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        
        print("✅ Environment loaded")

        # 3a. Define the custom backend configuration
        custom_controller_instance = NonlinearController(
            # We explicitly use the default (None) trajectory_file to trigger the hard-coded trajectory
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0]
        )

        # 3b. Define the Multirotor configuration
        config_multirotor = MultirotorConfig()
        config_multirotor.backends = [custom_controller_instance] # Use the custom controller instance

        # 3c. Direct Multirotor Instantiation
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [2.3, -1.5, 0.07], # Starting position slightly offset for the trajectory
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations are initialized
        self.world.reset()
        self.world.initialize_physics() 
        print("✅ Drone spawned and physics initialized.")

    def run(self):
        print("🚀 Starting simulation. The drone should immediately begin tracking the exponential trajectory.")
        self.timeline.play()
        
        # Run for 60 seconds or until simulation closes
        start_time = self.world.current_time
        max_run_time = 60.0

        while simulation_app.is_running() and (self.world.current_time - start_time) < max_run_time:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation finished.")
        self.timeline.stop()
        simulation_app.close()

# Main function to run the application
def main():
    pg_app = PegasusApp()
    pg_app.run()
    
if __name__ == "__main__":
    main()