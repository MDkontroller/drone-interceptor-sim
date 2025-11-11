# STEP 1: Initialize SimulationApp FIRST
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# STEP 2: NOW import everything else
import time
import omni.timeline
from omni.isaac.core.world import World
from pxr import Gf
from scipy.spatial.transform import Rotation
import inspect

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS

# ==============================================================================
# 1. INSPECT PX4MavlinkBackend CLASS METHODS
# (Confirmed to only have low-level methods, repeating for completeness)
# ==============================================================================
print("-" * 50)
print(f"1. PX4MavlinkBackend CLASS Available Methods:")
for name, member in inspect.getmembers(PX4MavlinkBackend):
    if not name.startswith('_') and callable(member):
        print(f"  - {name}")
print("-" * 50)
# ==============================================================================

def main():
    print("🚁 Starting Pegasus environment setup for inspection...")

    timeline = omni.timeline.get_timeline_interface()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    pg.world.reset()
    print("✅ Environment loaded")

    px4_mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "px4_autolaunch": True,
        "px4_dir": "/home/ubuntu/PX4-Autopilot",
        "px4_vehicle_model": pg.px4_default_airframe
    })
    px4_backend_instance = PX4MavlinkBackend(px4_mavlink_config)

    multirotor_config = MultirotorConfig()
    multirotor_config.backends = [px4_backend_instance]

    drone = Multirotor(
        "/World/quadrotor", ROBOTS['Iris'], 0, [0.0, 0.0, 0.07], 
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=multirotor_config,
    )
    pg.world.initialize_physics() 
    print("✅ Drone spawned")

    backend_handle = drone._backends[0] 
    timeline.play()

    print("⏳ Waiting 15 seconds for PX4 backend to boot before inspecting...")
    time.sleep(15) 
    
    # ==============================================================================
    # 2. FINAL ATTEMPT: INSPECT PX4MavlinkBackend INSTANCE ATTRIBUTES (Runtime)
    # Looking for a hidden 'vehicle' or 'commander' object containing the methods.
    # ==============================================================================
    print("-" * 50)
    print(f"2. PX4MavlinkBackend INSTANCE Available ATTRIBUTES (Runtime):")
    print(f"   Controller object type: {type(backend_handle)}")
    print("   Searching for: .commander, .mav_commander, .api, .control_interface...")
    
    # Inspecting all attributes, filtering out methods and non-public dunder names
    for name, member in inspect.getmembers(backend_handle):
        # Filter for attributes (not methods) and non-private names
        if not name.startswith('_') and not callable(member):
            print(f"  - {name}")
    print("-" * 50)
    # ==============================================================================

    timeline.stop()
    simulation_app.close()
    print("✅ Inspection complete.")

if __name__ == "__main__":
    main()
    simulation_app.close()