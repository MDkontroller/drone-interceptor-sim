import rasterio
from rasterio.warp import reproject, Resampling

# Read DEM to get target parameters
with rasterio.open('dem_utm10.tif') as dem:
    dem_bounds = dem.bounds
    dem_transform = dem.transform
    dem_width = dem.width
    dem_height = dem.height
    dem_crs = dem.crs
    
    print(f"DEM: {dem_width}x{dem_height} pixels")
    print(f"Bounds: {dem_bounds}")
    print(f"Pixel size: {dem_transform[0]:.2f}m")

# Read and align satellite
with rasterio.open('Browser_images/S2_RGB_raw.tif') as src:
    print(f"\nSatellite input: {src.width}x{src.height} pixels")
    
    # Create output with EXACT same parameters as DEM
    kwargs = src.meta.copy()
    kwargs.update({
        'crs': dem_crs,
        'transform': dem_transform,  # Use DEM's exact transform
        'width': dem_width,          # Use DEM's exact width
        'height': dem_height         # Use DEM's exact height
    })
    
    # Reproject
    with rasterio.open('RGB_aligned.tif', 'w', **kwargs) as dst:
        for i in range(1, src.count + 1):
            reproject(
                source=rasterio.band(src, i),
                destination=rasterio.band(dst, i),
                src_transform=src.transform,
                src_crs=src.crs,
                dst_transform=dem_transform,
                dst_crs=dem_crs,
                resampling=Resampling.bilinear
            )
    
    print(f"✅ Aligned RGB: {dem_width}x{dem_height} pixels")

# Verify
with rasterio.open('dem_utm10.tif') as dem:
    with rasterio.open('RGB_aligned.tif') as rgb:
        print(f"\n=== VERIFICATION ===")
        print(f"DEM size: {dem.width}x{dem.height}")
        print(f"RGB size: {rgb.width}x{rgb.height}")
        print(f"DEM bounds: {dem.bounds}")
        print(f"RGB bounds: {rgb.bounds}")
        print(f"\n✅ Sizes match: {dem.width == rgb.width and dem.height == rgb.height}")
        print(f"✅ Bounds match: {dem.bounds == rgb.bounds}")
        
        if dem.width == rgb.width and dem.height == rgb.height and dem.bounds == rgb.bounds:
            print("\n🎉 PERFECT ALIGNMENT! Ready for Isaac Sim!")
        else:
            print("\n⚠️ Not perfectly aligned")
