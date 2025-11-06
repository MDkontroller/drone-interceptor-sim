cat > data/README.md << 'EOF'
# Terrain Data Download & Processing Guide

This guide shows how to download and prepare terrain data for Isaac Sim drone simulation.

## Overview

We need two aligned files:
1. **DEM (Digital Elevation Model)** - Height data for 3D terrain geometry
2. **Satellite RGB** - Texture for realistic appearance

**Target Area:** Levi's Stadium, Santa Clara, CA (2.5km × 2.5km)  
**Coordinate System:** UTM Zone 10N (EPSG:32610)  
**Resolution:** 27m DEM, 10m satellite

---

## Prerequisites
```bash
# Install GDAL and Python tools
sudo apt install gdal-bin python3-gdal

# Install Python packages
pip install rasterio numpy matplotlib
```

---

## Step 1: Download DEM

**Source:** [OpenTopography - Copernicus GLO-30](https://portal.opentopography.org/)

### Configuration:

1. Go to OpenTopography
2. Select **Copernicus GLO-30 Digital Elevation Model**
3. Enter coordinates manually:
```
Xmin: -121.9842
Ymin: 37.3918
Xmax: -121.9558
Ymax: 37.4142
```

4. **Critical Setting:**
   - Output Coordinate System: **UTM Zone 10N (EPSG:32610)**
   - Format: **GeoTIFF**

5. Submit job and download

### Save File:
```bash
# Rename downloaded file
mv output.tif data/processed/dem_utm10.tif
```

---

## Step 2: Download Satellite Imagery

**Source:** [Copernicus Data Space Browser](https://browser.dataspace.copernicus.eu/)

### Upload AOI:

Use the provided GeoJSON file:
```bash
# Upload this file in the browser interface
data/aoi/levi_stadium.geojson
```

**GeoJSON Contents:**
```json
{
  "type": "FeatureCollection",
  "features": [{
    "type": "Feature",
    "properties": {"name": "Levi's Stadium 2.5km"},
    "geometry": {
      "type": "Polygon",
      "coordinates": [[
        [-121.9842, 37.4142],
        [-121.9558, 37.4142],
        [-121.9558, 37.3918],
        [-121.9842, 37.3918],
        [-121.9842, 37.4142]
      ]]
    }
  }]
}
```

### Download Configuration:

1. **Date:** July 2024 (pick clear day, < 10% clouds)
2. **Satellite:** Sentinel-2 L2A
3. **Tab:** Analytical (not Basic)
4. **Settings:**
   - Image format: **TIFF (8-bit)**
   - Resolution: **10m** or **HIGHEST**
   - Coordinate system: **UTM 10N (EPSG:32610)**
   - Layers (Raw): **B04** (Red), **B03** (Green), **B02** (Blue)

5. Download 3 band files

### Organize Files:
```bash
# Move to raw folder
mkdir -p data/raw/sentinel2
mv *_B04_*.tiff data/raw/sentinel2/B04_raw.tiff
mv *_B03_*.tiff data/raw/sentinel2/B03_raw.tiff
mv *_B02_*.tiff data/raw/sentinel2/B02_raw.tiff
```

---

## Step 3: Merge RGB Bands
```bash
cd data

# Merge into single 3-band RGB file
gdal_merge.py -separate \
  -o raw/sentinel2/S2_RGB_merged.tif \
  raw/sentinel2/B04_raw.tiff \
  raw/sentinel2/B03_raw.tiff \
  raw/sentinel2/B02_raw.tiff
```

**Output:** `data/raw/sentinel2/S2_RGB_merged.tif`

---

## Step 4: Align Satellite to DEM Grid
```bash
cd data

# Run alignment script
python << 'EOF'
import rasterio
from rasterio.warp import reproject, Resampling

print("Loading DEM parameters...")
with rasterio.open('processed/dem_utm10.tif') as dem:
    dem_transform = dem.transform
    dem_width = dem.width
    dem_height = dem.height
    dem_crs = dem.crs
    print(f"  DEM: {dem_width}x{dem_height} pixels")

print("\nAligning satellite to DEM grid...")
with rasterio.open('raw/sentinel2/S2_RGB_merged.tif') as src:
    profile = src.profile.copy()
    profile.update({
        'transform': dem_transform,
        'width': dem_width,
        'height': dem_height,
        'crs': dem_crs
    })
    
    with rasterio.open('processed/rgb_aligned.tif', 'w', **profile) as dst:
        for i in range(1, src.count + 1):
            reproject(
                source=rasterio.band(src, i),
                destination=rasterio.band(dst, i),
                src_transform=src.transform,
                dst_transform=dem_transform,
                resampling=Resampling.bilinear
            )
    
    print(f"  Output: {dem_width}x{dem_height} pixels")

print("\n✅ Alignment complete: processed/rgb_aligned.tif")
EOF
```

**Output:** `data/processed/rgb_aligned.tif`

---

## Step 5: Enhance Contrast (Optional)

Sentinel-2 raw bands are dark by default. Enhance for better visualization:
```bash
cd data

python << 'EOF'
import rasterio
import numpy as np

print("Enhancing contrast...")
with rasterio.open('processed/rgb_aligned.tif') as src:
    profile = src.profile
    rgb = np.array([src.read(i) for i in range(1, 4)])
    
    # Histogram stretch per band
    for i in range(3):
        band = rgb[i]
        valid = band[band > 0]
        if len(valid) > 0:
            p2, p98 = np.percentile(valid, (2, 98))
            rgb[i] = np.clip(255 * (band - p2) / (p98 - p2), 0, 255).astype('uint8')
    
    # Save enhanced version
    with rasterio.open('processed/rgb_aligned_enhanced.tif', 'w', **profile) as dst:
        for i in range(3):
            dst.write(rgb[i], i+1)

print("✅ Enhanced version: processed/rgb_aligned_enhanced.tif")
EOF
```

**Output:** `data/processed/rgb_aligned_enhanced.tif`

---

## Step 6: Verify Alignment
```bash
cd data

python << 'EOF'
import rasterio

print("=== Verification ===\n")

with rasterio.open('processed/dem_utm10.tif') as dem:
    print(f"DEM:")
    print(f"  Size: {dem.width}x{dem.height}")
    print(f"  Bounds: {dem.bounds}")
    print(f"  CRS: {dem.crs}")

with rasterio.open('processed/rgb_aligned_enhanced.tif') as rgb:
    print(f"\nRGB:")
    print(f"  Size: {rgb.width}x{rgb.height}")
    print(f"  Bounds: {rgb.bounds}")
    print(f"  CRS: {rgb.crs}")

print(f"\n✅ Sizes match: {dem.width == rgb.width and dem.height == rgb.height}")
print(f"✅ Bounds match: {dem.bounds == rgb.bounds}")
EOF
```

**Expected Output:**
```
=== Verification ===

DEM:
  Size: 94x91
  Bounds: BoundingBox(left=589881.37, bottom=4138849.44, right=592428.08, top=4141314.87)
  CRS: EPSG:32610

RGB:
  Size: 94x91
  Bounds: BoundingBox(left=589881.37, bottom=4138849.44, right=592428.08, top=4141314.87)
  CRS: EPSG:32610

✅ Sizes match: True
✅ Bounds match: True
```

---

## Final Files for Isaac Sim
```
data/processed/
├── dem_utm10.tif              # Height data (94×91 pixels, 27m/px)
└── rgb_aligned_enhanced.tif   # Satellite texture (94×91 pixels, enhanced)
```

**Both files:**
- ✅ Same dimensions (94×91 pixels)
- ✅ Same coordinate system (UTM Zone 10N)
- ✅ Same geographic bounds
- ✅ Pixel-perfect alignment

---

## Troubleshooting

### Problem: DEM downloaded in wrong CRS

**Symptom:** DEM has EPSG:4326 instead of EPSG:32610

**Fix:** Convert using gdalwarp
```bash
gdalwarp -t_srs EPSG:32610 -r bilinear \
  dem_original.tif \
  processed/dem_utm10.tif
```

### Problem: Satellite image is black

**Cause:** Raw Sentinel-2 values are dark (mean ~30-40 out of 255)

**Fix:** Run Step 5 (contrast enhancement)

### Problem: Sizes don't match

**Cause:** Alignment script didn't copy DEM's exact transform

**Fix:** Verify Step 4 uses `dem_transform` directly, not reconstructed bounds

---

## Quick Start (All Steps)
```bash
#!/bin/bash
# Complete terrain processing pipeline

cd data

# Step 1: Already done (manual download)

# Step 2: Already done (manual download)

# Step 3: Merge bands
gdal_merge.py -separate \
  -o raw/sentinel2/S2_RGB_merged.tif \
  raw/sentinel2/B04_raw.tiff \
  raw/sentinel2/B03_raw.tiff \
  raw/sentinel2/B02_raw.tiff

# Step 4 & 5: Align and enhance
python << 'PYTHON_EOF'
import rasterio
from rasterio.warp import reproject, Resampling
import numpy as np

# Align
with rasterio.open('processed/dem_utm10.tif') as dem:
    dem_transform = dem.transform
    dem_width = dem.width
    dem_height = dem.height
    
with rasterio.open('raw/sentinel2/S2_RGB_merged.tif') as src:
    profile = src.profile.copy()
    profile.update({
        'transform': dem_transform,
        'width': dem_width,
        'height': dem_height
    })
    
    with rasterio.open('processed/rgb_aligned.tif', 'w', **profile) as dst:
        for i in range(1, src.count + 1):
            reproject(
                source=rasterio.band(src, i),
                destination=rasterio.band(dst, i),
                src_transform=src.transform,
                dst_transform=dem_transform,
                resampling=Resampling.bilinear
            )

# Enhance
with rasterio.open('processed/rgb_aligned.tif') as src:
    profile = src.profile
    rgb = np.array([src.read(i) for i in range(1, 4)])
    
    for i in range(3):
        band = rgb[i]
        valid = band[band > 0]
        p2, p98 = np.percentile(valid, (2, 98))
        rgb[i] = np.clip(255 * (band - p2) / (p98 - p2), 0, 255).astype('uint8')
    
    with rasterio.open('processed/rgb_aligned_enhanced.tif', 'w', **profile) as dst:
        for i in range(3):
            dst.write(rgb[i], i+1)

print("✅ Processing complete!")
PYTHON_EOF
```

---

## Metadata

- **Location:** Levi's Stadium, Santa Clara, CA
- **Center:** 37.403°N, 121.970°W
- **Area:** 2.5km × 2.5km (~6.25 km²)
- **CRS:** UTM Zone 10N (EPSG:32610)
- **DEM Source:** Copernicus GLO-30
- **Satellite Source:** Sentinel-2 L2A (July 2024)
- **Processing Date:** 2025-11-05

EOF