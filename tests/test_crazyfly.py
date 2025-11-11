from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
from pxr import UsdPhysics, Gf
import omni.usd

print("🚁 Simple drone flight: A to B")

# Get current stage
stage = omni.usd.get_context().get_stage()

world = World()
world.scene.add_default_ground_plane()

# Add lighting
from pxr import UsdLux, Sdf
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(3000)

# Single drone
drone = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Drone",
        name="drone",
        position=np.array([0, 0, 2.0]),
        size=0.3,
        color=np.array([0, 1, 0])
    )
)

target_pos = np.array([20, 10, 3.0])

print(f"✅ Drone at A: {drone.get_world_pose()[0]}")
print(f"🎯 Target B: {target_pos}")

world.reset()

print("🚁 Flying from A to B...")

reached = False
for step in range(1000):
    current_pos, _ = drone.get_world_pose()
    
    direction = target_pos - current_pos
    distance = np.linalg.norm(direction)
    
    if distance < 0.5:
        if not reached:
            print(f"✅ REACHED TARGET at step {step}!")
            reached = True
        prim = stage.GetPrimAtPath("/World/Drone")
        UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
    else:
        velocity = (direction / distance) * 3.0
        prim = stage.GetPrimAtPath("/World/Drone")
        UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set(Gf.Vec3f(*velocity))
    
    world.step(render=True)
    
    if step % 100 == 0 and not reached:
        print(f"Step {step}: distance = {distance:.2f}m")

print("✅ Flight complete!")