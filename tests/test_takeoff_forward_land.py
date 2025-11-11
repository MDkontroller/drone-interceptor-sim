# 1. ADD THESE TWO LINES at the very top
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
#from pegasus.simulator.api import Simulator
#from pegasus.simulator import Simulator
from pegasus.simulator.core import Simulator
from pegasus.simulator.logic.vehicles import Multirotor
from pxr import Gf   # pxr solved by adding isaac sim import before execution
import time

# Create simulator
sim = Simulator(gui=True)

# Spawn the drone (simple PX4 quadcopter model)
drone = sim.spawn_vehicle(
    Multirotor,
    vehicle_name="drone1",
    position=Gf.Vec3d(0, 0, 0),
    px4_autostart=True
)

# Give PX4 a bit of time to boot
print("⏳ Waiting PX4 to initialize...")
time.sleep(5)

print("✅ Arming")
drone.arm()

print("⬆️ Taking off...")
drone.takeoff(altitude=3.0)    # Go up to 3 meters
time.sleep(6)

print("➡️ Moving forward 20 meters...")
drone.move_velocity(vx=2.0, vy=0.0, vz=0.0, duration=10)  # forward ~2m/s for 10s
time.sleep(10)

print("🛑 Hovering...")
drone.hover()
time.sleep(2)

print("⬇️ Landing...")
drone.land()
time.sleep(6)

print("🔒 Disarming")
drone.disarm()

print("✅ Flight test complete.")
