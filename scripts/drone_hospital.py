#!/usr/bin/env python
"""
Pegasus Drone Flight with Outdoor Environment
Uses Isaac Sim asset pack environments
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
from pxr import UsdGeom, Gf
import sys, os
import glob

# Pegasus imports
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import custom controller
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
utils_path = os.path.join(project_root, 'utils')
sys.path.insert(0, utils_path)
from nonlinear_controller import NonlinearController


class PegasusApp:
    """Pegasus app with outdoor environment and FPV camera."""

    def __init__(self):
        """Initialize the simulation environment."""
        
        # Acquire timeline
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus Interface
        self.pg = PegasusInterface()

        # Setup world
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Set GPS coordinates to Munich, Germany (for your hackathon location!)
        # Munich coordinates: 48.1351° N, 11.5820° E, ~520m altitude
        self.pg.set_global_coordinates(
            latitude=48.1351,
            longitude=11.5820,
            altitude=520.0
        )
        print("✅ GPS origin set to Munich, Germany")

        # Load Hospital environment - indoor but well-lit
        hospital_usd = os.path.expanduser(
            "~/isaac-sim-assets-environments/Assets/Isaac/5.1/Isaac/Environments/Hospital/hospital.usd"
        )
        
        if os.path.exists(hospital_usd):
            print(f"⏳ Loading Hospital environment...")
            try:
                self.pg.load_environment(hospital_usd)
                print(f"✅ Hospital environment loaded!")
                
                # Add extra lighting for better visibility
                self.add_lighting()
                
                # IMPORTANT: Position the viewport camera to see the environment
                self.position_viewport_camera()
                
            except Exception as e:
                print(f"⚠️  Failed to load Hospital: {e}")
                print(f"   Using Curved Gridroom...")
                self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        else:
            print(f"⏳ Loading Curved Gridroom...")
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
            print(f"✅ Environment loaded")

        # Create custom controller
        config_multirotor = MultirotorConfig()
        config_multirotor.backends = [NonlinearController(
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0]
        )]

        # Spawn drone at reasonable height
        self.drone = Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            [2.3, -1.5, 2.0],  # Start 2m up
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset simulation
        self.world.reset()
        
        # Add FPV camera
        self.add_fpv_camera()
        
        print("✅ Drone and camera initialized")
        print("📷 Switch to 'fpv_camera' in viewport to see FPV view")

    def position_viewport_camera(self):
        """Position the viewport camera to see the environment."""
        from omni.isaac.core.utils.viewports import set_camera_view
        
        # Position camera to get a good view of the scene
        # eye = where the camera is, target = where it's looking
        set_camera_view(
            eye=[50.0, 50.0, 30.0],  # Camera position (x, y, z)
            target=[0.0, 0.0, 0.0],  # Looking at origin
            camera_prim_path="/OmniverseKit_Persp"  # Default perspective camera
        )
        print("✅ Viewport camera positioned")
    
    def add_lighting(self):
        """Add proper lighting to the scene."""
        from pxr import UsdLux
        stage = omni.usd.get_context().get_stage()
        
        # Add dome light for ambient lighting
        dome_light_path = "/World/DomeLight"
        if not stage.GetPrimAtPath(dome_light_path):
            dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
            dome_light.CreateIntensityAttr(1000.0)
            print("✅ Added dome light")
        
        # Add distant light (sun) for directional lighting  
        distant_light_path = "/World/DistantLight"
        if not stage.GetPrimAtPath(distant_light_path):
            distant_light = UsdLux.DistantLight.Define(stage, distant_light_path)
            distant_light.CreateIntensityAttr(3000.0)
            distant_light.CreateAngleAttr(0.53)
            # Position the light
            xform = UsdGeom.Xformable(distant_light)
            xform.AddRotateXYZOp().Set(Gf.Vec3d(315, 45, 0))  # Angled down from above
            print("✅ Added distant light (sun)")

    def add_fpv_camera(self):
        """Add FPV camera to drone body."""
        stage = omni.usd.get_context().get_stage()
        
        camera_path = "/World/quadrotor1/body/fpv_camera"
        camera_prim = UsdGeom.Camera.Define(stage, camera_path)
        
        camera_prim.GetHorizontalApertureAttr().Set(20.955)
        camera_prim.GetVerticalApertureAttr().Set(15.2908)
        camera_prim.GetFocalLengthAttr().Set(18.0)
        camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
        
        xform = UsdGeom.Xformable(camera_prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(0.3, 0.0, 0.1))
        xform.AddRotateXYZOp().Set(Gf.Vec3d(-10, 90, 0))
        
        print(f"✅ FPV Camera: {camera_path}")

    def run(self):
        """Main simulation loop."""
        
        # Start simulation
        self.timeline.play()

        # Run simulation
        while simulation_app.is_running():
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