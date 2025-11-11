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
# 1. INSPECT MULTIROTOR CLASS (STATIC)
# ==============================================================================
print("-" * 50)
print(f"1. Multirotor CLASS Available Methods (Static):")
for name, member in inspect.getmembers(Multirotor):
    if not name.startswith('_') and callable(member):
        print(f"  - {name}")
print("-" * 50)
# ==============================================================================

def main():
    print("🚁 Starting Pegasus environment setup for inspection...")

    # Get timeline
    timeline = omni.timeline.get_timeline_interface()

    # Create Pegasus Interface
    pg = PegasusInterface()

    # Setup world
    pg._world = World(**pg._world_settings)
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    pg.world.reset()

    print("✅ Environment loaded")

    # 3. CONFIGURE AND SPAWN THE VEHICLE

    # 3a. Define the PX4 backend configuration
    px4_mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "px4_autolaunch": True,
        "px4_dir": "/home/ubuntu/PX4-Autopilot",
        "px4_vehicle_model": pg.px4_default_airframe
    })
    px4_backend_instance = PX4MavlinkBackend(px4_mavlink_config)

    # 3b. Define the Multirotor configuration
    multirotor_config = MultirotorConfig()
    multirotor_config.backends = [px4_backend_instance]

    # 3c. Direct Multirotor Instantiation
    drone = Multirotor(
        "/World/quadrotor",
        ROBOTS['Iris'],
        0,
        [0.0, 0.0, 0.07],
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=multirotor_config,
    )

    # Initialize physics after spawning the vehicle
    pg.world.initialize_physics() 

    print("✅ Drone spawned")

    # 4. GET THE BACKEND VEHICLE INTERFACE FOR CONTROL
    backend_handle = drone._backends[0] 
    controller = backend_handle.vehicle

    # Start simulation
    timeline.play()

    # Wait for PX4 (EXTENDED WAIT TIME)
    print("⏳ Waiting 15 seconds for PX4 backend to boot and inject methods...")
    time.sleep(15) 
    
    # ==============================================================================
    # 2. INSPECT CONTROLLER INSTANCE (RUNTIME) - THIS IS CRITICAL
    # ==============================================================================
    print("-" * 50)
    print(f"2. Controller INSTANCE Available Methods (Runtime):")
    print(f"   Controller object type: {type(controller)}")
    print("   Look for 'arm', 'takeoff', 'set_mode', or similar commands.")
    
    # Check if controller is still a Multirotor instance (recursive bug check)
    if type(controller).__name__ == 'Multirotor':
        print("\n   ⚠️ WARNING: Controller is recursively a Multirotor instance. Methods may be missing.")
        
    for name, member in inspect.getmembers(controller):
        if not name.startswith('_') and callable(member):
            print(f"  - {name}")
    print("-" * 50)
    # ==============================================================================

    # Stop simulation right after inspection
    timeline.stop()
    simulation_app.close()
    print("✅ Inspection complete.")

if __name__ == "__main__":
    main()
    # close sim app
    simulation_app.close()