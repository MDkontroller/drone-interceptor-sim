#!/usr/bin/env python
"""
Minimal Terrain Renderer - Simple approach that should definitely work
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
import numpy as np
from pxr import Gf, UsdGeom, UsdShade, Sdf, UsdLux
import rasterio
from PIL import Image
from pathlib import Path

def main():
    # Initialize
    timeline = omni.timeline.get_timeline_interface()
    world = World()
    stage = omni.usd.get_context().get_stage()
    
    print("✅ World initialized")
    
    # Load DEM
    dem_path = "/home/jet/drone-interceptor-sim/data/processed/dem_utm10.tif"
    with rasterio.open(dem_path) as dem:
        elevation = dem.read(1)
    
    print(f"📂 DEM loaded: {elevation.shape}")
    print(f"   Elevation: {elevation.min():.1f}m to {elevation.max():.1f}m")
    
    # Load RGB
    rgb_path = "/home/jet/drone-interceptor-sim/data/processed/rgb_enhanced.png"
    rgb_image = np.array(Image.open(rgb_path))
    
    print(f"📂 RGB loaded: {rgb_image.shape}")
    
    # Downsample
    downsample = 2
    elevation = elevation[::downsample, ::downsample]
    rgb_image = rgb_image[::downsample, ::downsample]
    height, width = elevation.shape
    
    print(f"🏔️  Creating mesh: {width} x {height}")
    
    # Create vertices
    vertices = []
    uvs = []
    scale = 2.0  # Make it bigger so we can see it
    
    for i in range(height):
        for j in range(width):
            x = float(j * scale)
            y = float(i * scale)
            z = float(elevation[i, j])  # Convert numpy.float32 to Python float!
            vertices.append(Gf.Vec3f(x, y, z))
            u = float(j / (width - 1))
            v = float(1 - i / (height - 1))
            uvs.append(Gf.Vec2f(u, v))
    
    # Create faces
    faces = []
    face_counts = []
    
    for i in range(height - 1):
        for j in range(width - 1):
            idx = i * width + j
            faces.extend([idx, idx + width, idx + 1])
            face_counts.append(3)
            faces.extend([idx + 1, idx + width, idx + width + 1])
            face_counts.append(3)
    
    print(f"   Triangles: {len(faces)//3}")
    
    # Create mesh
    mesh_path = "/World/terrain_mesh"
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(vertices)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    
    # Add UVs
    primvarsAPI = UsdGeom.PrimvarsAPI(mesh)
    st = primvarsAPI.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(uvs)
    
    # Center mesh
    mesh.AddTranslateOp().Set(Gf.Vec3d(-width * scale / 2, -height * scale / 2, 0))
    
    print("✅ Mesh created")
    
    # Create simple dome light using USD API directly
    light_path = "/World/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, light_path)
    dome_light.CreateIntensityAttr(1000.0)
    
    print("✅ Dome light created")
    
    # Save texture
    texture_path = "/tmp/terrain_tex.png"
    Image.fromarray(rgb_image).save(texture_path)
    print(f"✅ Texture saved: {texture_path}")
    print(f"   Texture size: {rgb_image.shape}")
    print(f"   Texture file exists: {Path(texture_path).exists()}")
    
    # Force app update to ensure texture is written
    simulation_app.update()
    
    # Create simple material
    mat_path = "/World/terrain_material"
    material = UsdShade.Material.Define(stage, mat_path)
    
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    
    # Texture shader
    tex_shader = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    tex_shader.CreateIdAttr("UsdUVTexture")
    tex_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    tex_shader.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    
    # Connect texture to shader
    diffuse_input = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    diffuse_input.ConnectToSource(tex_shader.ConnectableAPI(), "rgb")
    
    # Material properties
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    
    # Connect to material
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    
    # Bind material
    binding = UsdShade.MaterialBindingAPI(mesh.GetPrim())
    binding.Bind(material)
    
    print(f"✅ Material created and bound")
    print(f"   Material path: {mat_path}")
    print(f"   Texture path in material: {texture_path}")
    
    # Force update
    simulation_app.update()
    simulation_app.update()
    
    # Create camera
    camera_path = "/World/Camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(0, -200, 100))
    xform.AddRotateXYZOp().Set(Gf.Vec3d(30, 0, 0))
    
    print("✅ Camera created")
    print("\n" + "="*50)
    print("SELECT 'Camera' FROM VIEWPORT DROPDOWN!")
    print("TURN ON STAGE LIGHTS (sun icon)")
    print("="*50 + "\n")
    
    # Reset world
    world.reset()
    
    # Play
    timeline.play()
    
    # Run
    while simulation_app.is_running():
        world.step(render=True)
    
    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()