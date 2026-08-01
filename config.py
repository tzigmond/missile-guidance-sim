import numpy as np

STRAIGHT_TIME1 = 25
CURVE_TIME     = 25
STRAIGHT_TIME2 = 25

# ---------------------------------------------------------------------------
# Target aircraft: F-16C Fighting Falcon
#
# The F-16C is an agile 4th-generation air superiority fighter designed
# for sustained high-g maneuvering. Unlike the MiG-25 (which was optimized
# purely for speed), the F-16 was built to dogfight - making it a far more
# challenging target for SAM guidance systems.
#
# TARGET_SPEED: 540 m/s ≈ Mach 1.6 at altitude - combat cruise speed.
#   The F-16C typically operates at Mach 1.5-1.8 in a combat scenario.
#   Ref: USAF F-16 Flight Manual; Jane's All the World's Aircraft
#
# TARGET_MAX_SPEED: 690 m/s ≈ Mach 2.05 at altitude - maximum clean speed.
#   Ref: Museum of Aviation F-16A specs; Lockheed Martin F-16 documentation
#
# TARGET_MAX_G: 9g - F-16C structural limit.
#   The F-16 was specifically designed with a 9g airframe limit for
#   sustained high-g combat maneuvering.
#   Ref: USAF F-16 Flight Manual
# ---------------------------------------------------------------------------
TARGET_SPEED     = 540.0   # m/s - Mach ~1.6 at altitude, F-16C combat cruise
TARGET_MAX_SPEED = 690.0   # m/s - Mach ~2.05 at altitude, F-16C maximum clean
TARGET_MAX_G     = 9.0     # g   - F-16C structural limit

# ---------------------------------------------------------------------------
# Missile: AIM-120 AMRAAM class SAM
#
# MISSILE_SPEED: 1374 m/s ≈ Mach 4.0 at sea level - matches published
#   top speed of AIM-120 AMRAAM (Mach 4 capable).
#   Speed ratio ~2.54:1 (missile:target) - realistic SAM vs fighter engagement.
#   Ref: Jane's Air-Launched Weapons; globalsecurity.org AIM-120 specs
# ---------------------------------------------------------------------------
MISSILE_SPEED = 1374.0  # m/s - Mach ~4.0 at sea level (AIM-120 AMRAAM class)

# ---------------------------------------------------------------------------
# Missile physical properties
# Based on AIM-120 AMRAAM class tactical missile
# Ref: Jane's Air-Launched Weapons; globalsecurity.org AIM-120 specs
# ---------------------------------------------------------------------------
MISSILE_MASS = 152.0   # kg - launch mass (fixed, fuel burn not modeled yet)

# ---------------------------------------------------------------------------
# Motor boost phase
# SAM launches from dead stop - boost phase accelerates from 0 to MISSILE_SPEED
# Boost duration ~3s is representative of tactical SAM solid-fuel boosters.
# During boost, PN guidance is inactive - missile flies a preset vertical climb.
# Ref: Jane's Land-Based Air Defence; MIL-HDBK-1211(MI)
# ---------------------------------------------------------------------------
BOOST_TIME  = 3.0                            # s    - boost phase duration
BOOST_ACCEL = MISSILE_SPEED / BOOST_TIME     # m/s² - linear ramp to MISSILE_SPEED
                                              # Ref: MIL-HDBK-1211(MI)

# ---------------------------------------------------------------------------
# Target evasion parameters
# Models an F-16C pilot responding to a missile warning (RWR alert)
#
# Detection range: 25km - modern RWR systems detect SAM launches at 15-30km.
#   Ref: Jane's Avionics - Radar Warning Receivers
#
# Reaction time: 0.5s - trained pilot RWR response time.
#   Ref: MIL-HDBK-1472 - Human Factors Engineering of Military Systems
#
# Max g: 9g - F-16C sustained turn limit.
#   Ref: USAF F-16 Flight Manual
#
# Max speed: 690 m/s - F-16C maximum clean speed (Mach ~2.05).
#   Ref: Jane's All the World's Aircraft
# ---------------------------------------------------------------------------
TARGET_DETECTION_RANGE = 25000.0  # m   - missile detection range
TARGET_REACTION_TIME   = 0.5      # s   - pilot reaction delay before evasion
TARGET_MAX_G           = 9.0      # g   - F-16C structural limit (matches above)

# ---------------------------------------------------------------------------
# Evasion arc parameters
# Each evasion arc sweeps EVASION_TURN_ANGLE radians. Arcs are chained in
# alternating directions to create continuous S-turn evasion maneuvers.
# 120° (2π/3) gives a hard break that opens significant angular separation
# while keeping arc duration short enough to stress PN guidance each cycle.
# Ref: Shaw - Fighter Combat: Tactics and Maneuvering (1985), p.64-71
# ---------------------------------------------------------------------------
EVASION_TURN_ANGLE = np.pi * 2.0 / 3.0   # rad - 120° per arc

# ---------------------------------------------------------------------------
# Engagement geometry
# ---------------------------------------------------------------------------
TURN_ANGLE = -np.pi * 4 / 3
YZ_ANGLE   = -np.pi / 12

TMAX = 75
DT   = 0.001

MISSILE_START  = np.array([13000, 12000, 0],     dtype=float)
AIRCRAFT_START = np.array([0,     0,     12000], dtype=float)

MISSILE_LAUNCH_TIME = 0       # s - time when missile launches
KILL_DISTANCE       = 2       # m - kill radius

# ---------------------------------------------------------------------------
# Vertical drift rate during banked turn
# Adds realistic climb/descent through the turn maneuver.
# Matches reference implementation in trace_and_chase.py (read-only).
# ---------------------------------------------------------------------------
CLIMB_RATE_CURVE = -0.001

# ---------------------------------------------------------------------------
# Missile lateral g-force limit
# Modern radar/IR-guided SAMs typically operate at 20-40g lateral.
# Ref: MIL-HDBK-1211 (1995)
# ---------------------------------------------------------------------------
MAX_G_FORCE = 40              # g - missile lateral acceleration limit
MAX_ACCEL   = MAX_G_FORCE * 9.80665  # m/s²

# ---------------------------------------------------------------------------
# Physical constants
# Ref: U.S. Standard Atmosphere 1976 (NOAA/NASA), Table 2
# ---------------------------------------------------------------------------
GRAVITY = 9.80665             # m/s²

# ---------------------------------------------------------------------------
# Proportional Navigation constant
# Typical effective navigation ratio: 3-5
# Ref: DTIC ADP010953
# ---------------------------------------------------------------------------
NAV_CONSTANT = 5.0