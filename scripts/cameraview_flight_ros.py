#!/usr/bin/env python
"""
Pegasus Drone with Hospital Environment + ROS2 Publishing
Publishes drone state, sensors, and camera to ROS2 topics
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation
from pxr import UsdGeom, Gf, UsdLux
import sys, os

# Enable ROS2 extension
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

# Pegasus imports
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

# Import custom controller
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
utils_path = os.path.join(project_root, 'utils')
sys.path.insert(0, utils_path)
from nonlinear_controller import NonlinearController


class PegasusApp:
    """Pegasus app with Hospital environment, FPV camera, and ROS2 publishing."""

    def __init__(self):
        """Initialize the simulation environment."""
        
        # Acquire timeline
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus Interface
        self.pg = PegasusInterface()

        # Setup world
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Set GPS coordinates to Munich
        self.pg.set_global_coordinates(
            latitude=48.1351,
            longitude=11.5820,
            altitude=520.0
        )
        print("✅ GPS origin set to Munich, Germany")

        # Load Hospital environment
        hospital_usd = os.path.expanduser(
            "~/isaac-sim-assets-environments/Assets/Isaac/5.1/Isaac/Environments/Hospital/hospital.usd"
        )
        
        if os.path.exists(hospital_usd):
            print(f"⏳ Loading Hospital environment (this may take a minute)...")
            try:
                self.pg.load_environment(hospital_usd)
                print(f"✅ Hospital environment loaded!")
                
                # Add lighting
                self.add_lighting()
                
                # Position camera
                self.position_viewport_camera()
                
            except Exception as e:
                print(f"⚠️  Failed to load Hospital: {e}")
                print(f"   Using Curved Gridroom...")
                self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        else:
            print(f"⏳ Loading Curved Gridroom...")
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Create multirotor configuration with BOTH custom controller AND ROS2
        config_multirotor = MultirotorConfig()
        
        # Add custom nonlinear controller
        config_multirotor.backends = [
            NonlinearController(
                Ki=[0.5, 0.5, 0.5],
                Kr=[2.0, 2.0, 2.0]
            ),
            # Add ROS2 backend for publishing
            ROS2Backend(
                vehicle_id=1,
                config={
                    "namespace": 'drone',
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": True,
                    "pub_tf": True,  # Publish transforms
                    "sub_control": False  # Don't subscribe to control (we use custom controller)
                }
            )
        ]
        
        # Add camera sensor for ROS2
        config_multirotor.graphical_sensors = [
            
            MonocularCamera("fpv_camera", 
        config={
            "update_rate": 30.0,
            "resolution": [640, 480]  # Add resolution
        })
        ]

        # Spawn drone
        self.drone = Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            [2.3, -1.5, 2.0],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset simulation
        self.world.reset()
        
        # Add FPV camera to USD (for Isaac Sim viewport)
        self.add_fpv_camera_usd()
        
        print("✅ Drone initialized with ROS2 publishing")
        print("\n" + "="*60)
        print("📡 ROS2 Topics Available:")
        print("   - /drone/state (drone position, velocity, attitude)")
        print("   - /drone/sensors (IMU, GPS, etc.)")
        print("   - /drone/fpv_camera/image (camera feed)")
        print("   - /tf (transforms)")
        print("\n💡 To view topics, open a new terminal and run:")
        print("   ros2 topic list")
        print("   ros2 topic echo /drone/state")
        print("="*60 + "\n")

    def position_viewport_camera(self):
        """Position the viewport camera to see the environment."""
        from omni.isaac.core.utils.viewports import set_camera_view
        
        set_camera_view(
            eye=[50.0, 50.0, 30.0],
            target=[0.0, 0.0, 0.0],
            camera_prim_path="/OmniverseKit_Persp"
        )
        print("✅ Viewport camera positioned")
    
    def add_lighting(self):
        """Add proper lighting to the scene."""
        stage = omni.usd.get_context().get_stage()
        
        # Dome light
        dome_light_path = "/World/DomeLight"
        if not stage.GetPrimAtPath(dome_light_path):
            dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
            dome_light.CreateIntensityAttr(1000.0)
            print("✅ Added dome light")
        
        # Distant light (sun)
        distant_light_path = "/World/DistantLight"
        if not stage.GetPrimAtPath(distant_light_path):
            distant_light = UsdLux.DistantLight.Define(stage, distant_light_path)
            distant_light.CreateIntensityAttr(3000.0)
            distant_light.CreateAngleAttr(0.53)
            xform = UsdGeom.Xformable(distant_light)
            xform.AddRotateXYZOp().Set(Gf.Vec3d(315, 45, 0))
            print("✅ Added distant light")

    def add_fpv_camera_usd(self):
        """Add FPV camera to USD for Isaac Sim viewport viewing."""
        stage = omni.usd.get_context().get_stage()
        
        camera_path = "/World/quadrotor1/body/fpv_camera_viewport"
        camera_prim = UsdGeom.Camera.Define(stage, camera_path)
        
        camera_prim.GetHorizontalApertureAttr().Set(20.955)
        camera_prim.GetVerticalApertureAttr().Set(15.2908)
        camera_prim.GetFocalLengthAttr().Set(18.0)
        camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
        
        xform = UsdGeom.Xformable(camera_prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(0.3, 0.0, 0.1))
        xform.AddRotateXYZOp().Set(Gf.Vec3d(-10, 90, 0))
        
        print(f"✅ FPV viewport camera: {camera_path}")

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