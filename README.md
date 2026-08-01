# 3D Missile-Aircraft Pursuit Simulation

A Python simulation and 3D visualization of an AIM-120 AMRAAM class missile pursuing an F-16C Fighting Falcon, using Proportional Navigation (PN) guidance.

The missile launches from dead stop, boosts to Mach 4, then steers using PN guidance toward a maneuvering target. The F-16C flies a preset three-segment path until its RWR detects the missile, then transitions into chained S-turn evasion arcs. The full engagement is rendered as an animated 3D plot with a real-time HUD.

## Features

- **Proportional Navigation guidance (3D)** - commands lateral acceleration proportional to LOS rotation rate; navigation constant N=5 (Ref: DTIC ADP010953)
- **Boost phase** - missile launches from dead stop and ramps to full speed over 3 seconds; PN inactive during boost
- **Gravity and g-limiting** - gravity applied during sustain phase; lateral acceleration capped at 40g (Ref: MIL-HDBK-1211)
- **F-16C Fighting Falcon target** - Mach 1.6 cruise, 9g structural limit, realistic engagement geometry
- **Active evasion with RWR** - target detects missile at 25 km, reacts after 0.5 s, then executes continuous chained 120° S-turn arcs (Ref: Shaw - *Fighter Combat*, 1985)
- **Missile state machine** - distinct FLYING / HIT / MISSED states; miss detected by negative closing velocity
- **Simulation summary** - prints peak lateral g, result (INTERCEPT or MISS), intercept time, and final miss distance
- **3D animated visualization** - real-time position trails, HUD with time and range readout, intercept star marker
- **Switchable color themes** - light bone theme active; dark tactical theme available in `visualization/animation.py`

## Project Structure

```
missile-guidance-sim/
├── main.py                        # Entry point
├── config.py                      # All scenario parameters
├── guidance/
│   └── proportional_navigation.py # 3D PN guidance law
├── simulation/
│   ├── environment.py             # Simulation loop and result reporting
│   ├── missile.py                 # Missile kinematics and state machine
│   └── target.py                  # F-16C trajectory and evasion logic
└── visualization/
    └── animation.py               # Matplotlib 3D animation and HUD
```

## Requirements

- Python 3.7+
- NumPy
- Matplotlib

## Usage

```bash
python main.py
```

The simulation runs first, prints a summary, then opens the 3D animation.

## Parameters

All scenario parameters are in `config.py`:

| Parameter | Default | Description |
|---|---|---|
| `TARGET_SPEED` | 540 m/s | F-16C combat cruise speed (~Mach 1.6) |
| `TARGET_MAX_G` | 9 g | F-16C structural g limit |
| `MISSILE_SPEED` | 1374 m/s | AIM-120 class top speed (~Mach 4) |
| `BOOST_TIME` | 3 s | Boost phase duration |
| `MAX_G_FORCE` | 40 g | Missile lateral acceleration limit |
| `NAV_CONSTANT` | 5.0 | PN navigation ratio (typical range 3-5) |
| `TARGET_DETECTION_RANGE` | 25000 m | RWR detection range |
| `TARGET_REACTION_TIME` | 0.5 s | Pilot reaction delay after RWR alert |
| `EVASION_TURN_ANGLE` | 120° | Sweep angle per S-turn arc |
| `KILL_DISTANCE` | 2 m | Intercept (kill) radius |
| `MISSILE_START` | (13000, 12000, 0) | Missile launch position [m] |
| `AIRCRAFT_START` | (0, 0, 12000) | Aircraft initial position [m] |
| `TMAX` / `DT` | 75 s / 0.001 s | Simulation duration and timestep |

## Output

The simulation prints a summary on completion:

```
====================================================
  SIMULATION SUMMARY
====================================================
  Missile launch time :  0.00 s
  Target speed        :  540.0 m/s
  Missile speed       :  1374.0 m/s
  Nav constant (N)    :  5.0
  Peak lateral g      :  XX.X g
  Result              :  INTERCEPT at t=XX.XXs
====================================================
```

The 3D animation shows:
- Aircraft and missile flight paths with position trails
- Real-time range (D) and elapsed time (T) HUD
- Start position markers for both platforms
- Intercept star marker on hit

## References

- DTIC ADP010953 - Proportional Navigation guidance law
- MIL-HDBK-1211(MI) - Missile flight simulation methods
- Jane's Air-Launched Weapons - AIM-120 AMRAAM specifications
- Jane's All the World's Aircraft - F-16C performance data
- Shaw, R.L. - *Fighter Combat: Tactics and Maneuvering* (1985)
- Zarchan - *Tactical and Strategic Missile Guidance* (2012)

---

Built as a demonstration of simplified aerospace pursuit dynamics and proportional navigation guidance. Contributions and scenario expansions are welcome.
