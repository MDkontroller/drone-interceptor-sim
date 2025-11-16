# Robust Aerial Interception Under Unreliable Detection

This repository contains a simulation to evaluate interception strategies using two controllers:
- **PID** (baseline “chase-the-target” controller)
- **Proportional Navigation (PN)** (geometry-based interception controller)

The full simulation is implemented in:

```bash
montecarlo_PID_PN.py
```


The script runs multiple randomized interception scenarios (Monte Carlo) and compares the success rates of PID and PN under noisy detections filtered by a Kalman Filter.

---

## Environment Setup (`.visenv`)

A minimal environment is provided via the `.visenv` virtual environment folder.

### Create & activate the environment:

```bash
python3 -m venv .visenv
source .visenv/bin/activate
pip install numpy matplotlib scipy
```

To deactivate:
```bash
deactivate
```

## Running the Simulation

Once inside the environment:

```bash
source .visenv/bin/activate
python montecarlo_PID_PN.py
```

This will:

- Sample random initial conditions
- Simulate noisy detection data
- Apply a Kalman Filter to estimate target position/velocity
- Run both PID and PN controllers
- Determine interception success
- Print aggregated success rates
- Optionally show trajectory plots (if enabled)