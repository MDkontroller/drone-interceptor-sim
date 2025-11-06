import rasterio
import matplotlib.pyplot as plt
import numpy as np

# Read DEM
with rasterio.open('dem_utm10.tif') as dem:
    elevation = dem.read(1)
    
# Read RGB
with rasterio.open('RGB_aligned.tif') as rgb:
    red = rgb.read(1)
    green = rgb.read(2)
    blue = rgb.read(3)
    
    # Stack into RGB image
    rgb_image = np.dstack([red, green, blue]).astype(float)
    
    print(f"Original ranges: R:{red.min()}-{red.max()}, G:{green.min()}-{green.max()}, B:{blue.min()}-{blue.max()}")
    print(f"Original means: R:{red.mean():.1f}, G:{green.mean():.1f}, B:{blue.mean():.1f}")
    
    # Method 1: Simple stretch (0-255 → 0-1)
    rgb_simple = rgb_image / 255.0
    
    # Method 2: Histogram stretch (enhance contrast)
    p2, p98 = np.percentile(rgb_image[rgb_image > 0], (2, 98))
    rgb_stretched = np.clip((rgb_image - p2) / (p98 - p2), 0, 1)
    
    # Method 3: Gamma correction (brighten)
    gamma = 0.5  # <1 = brighter, >1 = darker
    rgb_gamma = np.power(rgb_image / 255.0, gamma)

# Plot all versions
fig, axes = plt.subplots(2, 2, figsize=(14, 12))

axes[0, 0].imshow(elevation, cmap='terrain')
axes[0, 0].set_title('DEM - Elevation', fontsize=14, fontweight='bold')
axes[0, 0].axis('off')

axes[0, 1].imshow(rgb_simple)
axes[0, 1].set_title('RGB - Original (Too Dark)', fontsize=14)
axes[0, 1].axis('off')

axes[1, 0].imshow(rgb_stretched)
axes[1, 0].set_title('RGB - Histogram Stretch (Best)', fontsize=14, fontweight='bold')
axes[1, 0].axis('off')

axes[1, 1].imshow(rgb_gamma)
axes[1, 1].set_title('RGB - Gamma Corrected', fontsize=14)
axes[1, 1].axis('off')

plt.tight_layout()
plt.savefig('terrain_preview_corrected.png', dpi=200, bbox_inches='tight')
print("\n✅ Corrected preview saved: terrain_preview_corrected.png")
plt.close()

# Also save the best version standalone
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

ax1.imshow(elevation, cmap='terrain')
ax1.set_title('DEM - Elevation (meters)', fontsize=14, fontweight='bold')
ax1.axis('off')

ax2.imshow(rgb_stretched)
ax2.set_title('Satellite RGB - Aligned & Enhanced', fontsize=14, fontweight='bold')
ax2.axis('off')

plt.tight_layout()
plt.savefig('terrain_final.png', dpi=200, bbox_inches='tight')
print("✅ Final preview saved: terrain_final.png")