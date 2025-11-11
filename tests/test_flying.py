from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere
import numpy as np
from pxr import UsdPhysics, Gf

print("🚁 Creating world...")
world = World()
world.scene.add_default_ground_plane()

# Create 3 defender drones with UNIQUE names
defenders = []
for i in range(3):
    angle = i * 2 * np.pi / 3
    x, y = 5 * np.cos(angle), 5 * np.sin(angle)
    
    defender = world.scene.add(
        DynamicSphere(
            prim_path=f"/World/Defender_{i}",
            name=f"defender_{i}",  # UNIQUE name for each
            position=np.array([x, y, 2.0]),
            radius=0.3,
            color=np.array([0, 1, 0])
        )
    )
    defenders.append(defender)

# Create target drone
target = world.scene.add(
    DynamicSphere(
        prim_path="/World/Target",
        name="target",
        position=np.array([15, 0, 4.0]),
        radius=0.2,
        color=np.array([1, 0, 0])
    )
)

print("✅ Drones created")
world.reset()

stage = world.stage

print("🎯 Starting pursuit...")
for step in range(1500):
    tgt_pos, _ = target.get_world_pose()
    
    # Pursuit control
    for i, defender in enumerate(defenders):
        def_pos, _ = defender.get_world_pose()
        direction = tgt_pos - def_pos
        distance = np.linalg.norm(direction)
        
        if distance > 1.0:
            velocity = (direction / distance) * 3.0
            prim = stage.GetPrimAtPath(f"/World/Defender_{i}")
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_body_api.GetVelocityAttr().Set(Gf.Vec3f(float(velocity[0]), float(velocity[1]), float(velocity[2])))
    
    # Target evasion
    evasion = np.array([2.0*np.sin(step*0.05), 2.0*np.cos(step*0.05), 0.3*np.sin(step*0.02)])
    target_prim = stage.GetPrimAtPath("/World/Target")
    UsdPhysics.RigidBodyAPI(target_prim).GetVelocityAttr().Set(Gf.Vec3f(*evasion))
    
    world.step(render=True)
    
    if step % 200 == 0:
        min_dist = min([np.linalg.norm(d.get_world_pose()[0] - tgt_pos) for d in defenders])
        print(f"Step {step}: distance={min_dist:.2f}m")

print("✅ Complete")
simulation_app.close()