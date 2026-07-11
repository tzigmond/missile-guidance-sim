# 3D Missile–Aircraft Pursuit Simulation

A Python simulation and 3D visualization of a missile pursuing an aircraft, using proportional navigation guidance.

The aircraft flies a multi-segment trajectory (straight → 3D turn → straight), while the missile continuously steers toward the aircraft's instantaneous position. The full engagement is rendered as an animated 3D plot with real-time position and intercept-distance tracking.

## Features

- **Multi-segment target trajectory** — straight flight, 3D turn, and final straight segment
- **Adaptive missile guidance** — missile steers toward the aircraft's instantaneous position
- **Configurable scenario** — speeds, launch positions, kill distance, turn geometry, and segment timing
- **3D animated visualization** using Matplotlib
- **Real-time intercept distance and marker display**
- Suitable for aerospace research or educational use

## Requirements

- Python 3.7+
- NumPy
- Matplotlib
- SciPy

## Usage

Run the script. It will:

- Simulate both missile and aircraft trajectories
- Print key simulation events (missile launch, intercept)
- Visualize the pursuit in a 3D Matplotlib animation

## Parameters

Key variables exposed for scenario customization (edit directly in the script):

| Variable | Description |
|---|---|
| `straight_time`, `curve_time`, `straight_time2` | Segment durations |
| `targ_vel`, `miss_vel` | Aircraft and missile speeds |
| `turn_angle`, `kill_dist`, `climb_rate_curve` | Turn geometry and intercept threshold |

## Output

On completion, the simulation displays:

- Aircraft and missile flight paths in 3D
- Instantaneous positions and distances throughout the engagement
- Start and intercept points, marked for clarity

---

Built as a demonstration of simplified aerospace pursuit dynamics and proportional navigation guidance. Contributions and scenario expansions are welcome.
