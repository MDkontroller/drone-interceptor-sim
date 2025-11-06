cat > README.md << 'EOF'
# Drone Interceptor - AI-Powered Counter-Drone System. 🚀

**Autonomous drone interceptor trained via Reinforcement Learning in photorealistic terrain simulation.**

![Terrain Visualization](data/preview/terrain_final.png)

---

## Overview

Vision-based interceptor drone that autonomously detects, tracks, and neutralizes hostile drones using:
- **Perception:** YOLOv8 object detection
- **Decision:** RL policy (PPO) trained in Isaac Sim
- **Control:** PX4 Autopilot with MAVLink
- **Environment:** 2.5km² photorealistic terrain (Levi's Stadium, CA)

**Hackathon:** Hack the Impossible 2025 (San Francisco, Nov 15-17)  
**Team:** Alessandro Tuccillo + Nicolas Sarmiento

---

## Prerequisites


See [DEVELOPMENT.md](DEVELOPMENT.md) for:
- Installing `uv` (Python package manager)
- Setting up the development environment
- Adding dependencies
- all very simple stuff

**Already have `uv`?** Just run:
```bash
uv sync  # Install all dependencies
```
---



## Quick Start

### 1. Setup Terrain Data

Download DEM and Sentinel-2 imagery (manual), then process to generate terrain files for Isaac Sim.

See detailed instructions: **[data/README.md](data/README.md)**

### 2. Setup Isaac Sim Environment  (coming soon)
```bash
# Install Isaac Sim + Isaac Lab
# See: docs/isaac_sim_setup.md
```
### 3. Run Simulation (coming soon)
```bash
# Train RL policy
python src/train.py

# Run demo
python src/demo.py
```

If you want to test the current perception and control stack without Isaac Sim,
you can use the included sample video ('data/test_drone.mp4'):

```bash
# Run Perception + Control on Sample Video
python -m main --video data/test_drone.mp4 --show --fps 15
```

This command will:
- Run YOLOv8-Nano detection and Kalman Filter tracking in real time
- Estimate target range and LOS direction
- Compute Proportional-Navigation (PN) guidance commands (TBD with sim!)
- Overlay bounding boxes and a live HUD with telemetry

---

## System Architecture
```
┌─────────────────────────────────────────────┐
│ Camera → YOLO Detection → RL Policy → PX4  │
└─────────────────────────────────────────────┘
         ↓              ↓            ↓
   Target Bbox    Velocity Cmd   Motor Cmd
                       ↓
              Isaac Sim Physics
              (GPU-accelerated)
```

---

## Project Structure (Approximate)
```
geosim/
├── data/                    # Terrain data (DEM + satellite)
│   ├── README.md           # ⭐ Start here for data setup
│   ├── processed/          # Final Processing scripts
├── src/                    # Isaac Sim integration
│   ├── terrain_loader.py   # examples
│   ├── drone_env.py
│   └── train.py
│
├── notebooks/              # Exploration & analysis
└── README.md              # This file
```

---

## Key Features

- ✅ **Photorealistic terrain** from real satellite data (Copernicus Sentinel-2)
- ✅ **GPU-accelerated physics** (4096 parallel environments, estimated)
- ✅ **Deployable control stack** (PX4 Autopilot)
- ✅ **Vision-based detection** (YOLOv8-Nano @ 10Hz)
- ✅ **RL-trained policy** (PPO with 256→128→64 network)

---

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Simulation | NVIDIA Isaac Sim 5.1 + Isaac Lab |
| Physics | PhysX (CUDA-accelerated) |
| Flight Stack | PX4 Autopilot + Pegasus Simulator |
| ML Training | RSL-RL (PPO algorithm) |
| Perception | YOLOv8-Nano |
| Terrain Data | Copernicus GLO-30 DEM + Sentinel-2 |

---

## Demo Scenarios

1. **Basic Interception** - Straight-line target (90% success)
2. **Evasive Target** - Zigzag maneuvers (75% success)
3. **Multi-Drone** - Coordinated swarm attack (stretch goal)

---

## Development Timeline

- **Nov 5-7:** Infrastructure setup (terrain, Isaac Sim, PX4)
- **Nov 8-14:** Algorithm development (YOLO, RL training)
- **Nov 15-17:** Hackathon demo and presentation

---

## License

MIT License - See [LICENSE](LICENSE) for details

---

## Resources

- **Terrain Data:** ESA Copernicus Programme
- **Simulation:** NVIDIA Isaac Sim
- **Flight Stack:** PX4 Autopilot, Pegasus Simulator

---

**Status:** 🟢 Terrain data complete | 🟡 Isaac Sim setup in progress

For detailed terrain setup: **[data/README.md](data/README.md)**
EOF