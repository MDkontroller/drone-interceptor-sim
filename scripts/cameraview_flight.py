#!/usr/bin/env python
"""
Pegasus Drone with FPV Camera

TODO: 

camera needs to be repositioned. currently in the worng axis, camera field of view 
can be adjusted.

"""

 

# Start Isaac Sim
import carb
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Now import everything else
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
from pxr import UsdGeom, Gf
import sys, os

# Pegasus imports
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import custom controller - FIX THE PATH
# Go up one level from scripts/ to get to project root, then into utils/
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)  # Go up one level
utils_path = os.path.join(project_root, 'utils')
sys.path.insert(0, utils_path)

from nonlinear_controller import NonlinearController


class PegasusApp:
    """
    Pegasus app with FPV camera attached to drone
    """

    def __init__(self):
        """Initialize the simulation environment."""
        
        # Acquire timeline
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus Interface
        self.pg = PegasusInterface()

        # Setup world
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load environment
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        print("✅ Environment loaded")

        # Create custom controller
        config_multirotor = MultirotorConfig()
        config_multirotor.backends = [NonlinearController(
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0]
        )]

        # Spawn drone
        self.drone = Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            [2.3, -1.5, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset simulation - this initializes all robots
        self.world.reset()
        
        # Add camera AFTER reset
        self.add_fpv_camera()
        
        print("✅ Drone and camera initialized")

    def add_fpv_camera(self):
        """Add FPV camera to drone body."""
        stage = omni.usd.get_context().get_stage()
        
        # Create camera as child of drone body
        camera_path = "/World/quadrotor1/body/fpv_camera"
        
        camera_prim = UsdGeom.Camera.Define(stage, camera_path)
        
        # Camera properties
        camera_prim.GetHorizontalApertureAttr().Set(20.955)
        camera_prim.GetVerticalApertureAttr().Set(15.2908)
        camera_prim.GetFocalLengthAttr().Set(18.0)  # Wide FOV
        camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
        
        # Position camera (forward and up from body center)
        xform = UsdGeom.Xformable(camera_prim)
        
        # Position: 0.5m forward (X), 0m right (Y), 0.2m up (Z)
        xform.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.2))
        
        # Rotation: The camera default looks down -Z axis in its local frame
        # To look forward in drone frame, we need to rotate 90° around Y axis (pitch up)
        # Then optionally tilt down slightly for FPV view
        # Order: X (roll), Y (pitch), Z (yaw)
        xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 90, 0))  # Pitch 90° to look forward
        
        print(f"✅ FPV Camera created at: {camera_path}")
        print("📷 Camera pointing forward (90° pitch rotation)")
        print("   To view: Window → Viewport → Viewport 2 → select 'fpv_camera'")

    def run(self):
        """Main simulation loop."""
        
        # Start simulation
        self.timeline.play()

        # Run simulation
        while simulation_app.is_running():
            # Physics step
            self.world.step(render=True)
        
        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()