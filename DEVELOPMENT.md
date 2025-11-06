# Development Setup

## Prerequisites

- **Python 3.11+**
- **uv** (fast Python package manager)
- **GDAL** (for terrain processing)

---

## 1. Install `uv` (Python Package Manager)
```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc

# Verify installation
uv --version
```

---

## 2. Clone Repository & Setup Environment
```bash
# Clone repo
git clone <repo-url>
cd geosim

# Create virtual environment (Python 3.11)
uv venv --python 3.11

# Activate environment
source .venv/bin/activate  # Linux/Mac
# or
.venv\Scripts\activate     # Windows
```

---

## 3. Install Dependencies
```bash
# Install all project dependencies
uv sync

# Or install from scratch
uv add rasterio numpy matplotlib gdal
```

---

## 4. IDE Setup (VSCode/Cursor)

### Option A: Automatic
```bash
# VSCode should auto-detect .venv
# If not, proceed to Option B
```

### Option B: Manual Python Path
```bash
# 1. Find Python path
uv run which python
# Output: /home/username/repos/geosim/.venv/bin/python

# 2. In VSCode/Cursor:
#    - Cmd/Ctrl + Shift + P
#    - "Python: Select Interpreter"
#    - Enter path from step 1
```

---

## 5. Common Commands

### Add New Package
```bash
uv add <package-name>
# Example:
uv add torch torchvision
```

### Run Python Script
```bash
# Option 1: With uv (auto-activates venv)
uv run python script.py

# Option 2: After manual activation
python script.py
```

### Update Dependencies
```bash
uv sync --upgrade
```

### Remove Package
```bash
uv remove <package-name>
```

---

## 6. Install System Dependencies

### GDAL (Required for terrain processing)
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install gdal-bin python3-gdal

```

---

## Project-Specific Setup

### Terrain Data Processing
```bash
cd data
uv run python scripts/02_align_terrain.py
```

### Isaac Sim (Coming Soon)
```bash
# See docs/isaac_sim_setup.md
```

---

## Troubleshooting

**Problem:** `uv: command not found`
```bash
# Re-run install and source
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
```

**Problem:** GDAL import error
```bash
# Install system GDAL first
sudo apt install gdal-bin python3-gdal
# Then reinstall Python bindings
uv add gdal --reinstall
```

**Problem:** Wrong Python version
```bash
# Recreate venv with correct version
rm -rf .venv
uv venv --python 3.11
uv sync
```

---

## Why `uv`?

- ⚡ **10-100x faster** than pip
- 🔒 **Automatic dependency resolution**
- 📦 **Built-in virtual environment management**
- 🎯 **Single tool** for package management

[Link about uv](https://github.com/astral-sh/uv)

---

**Next Steps:** See [data/README.md](data/README.md) for terrain setup instructions.