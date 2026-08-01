import numpy as np
import config
from guidance.proportional_navigation import proportional_navigation

# MissileState enum for clean state machine
# Ref: MIL-HDBK-1211(MI) - missile flight phases
from enum import Enum

class MissileState(Enum):
    FLYING   = "flying"
    HIT      = "hit"
    MISSED   = "missed"


class Missile:

    def __init__(self):
        self.position = config.MISSILE_START.copy().astype(float)
        self.mass     = config.MISSILE_MASS

        # SAM launches from dead stop - speed builds during boost phase
        # Ref: MIL-HDBK-1211(MI) - initial conditions
        self.speed    = 0.0

        # Initial velocity: small upward nudge so vectors are valid on first step.
        # Boost phase immediately overrides this with vertical climb.
        # Ref: MIL-HDBK-1211(MI)
        self.velocity = np.array([0.0, 0.0, 1.0]) * 0.001

        self.state        = MissileState.FLYING
        self.peak_g       = 0.0
        self._flight_time = 0.0   # tracks time since launch for motor phase logic

    @property
    def active(self):
        return self.state == MissileState.FLYING

    def step(self, target_pos, target_vel, dt):
        """
        Advance the missile state by one timestep.

        PARAMETERS
        ----------
        target_pos : np.array (3,)
            Current target position [m]
        target_vel : np.array (3,)
            Current target velocity [m/s]
        dt : float
            Simulation timestep [s]

        RETURNS
        -------
        position : np.array (3,)
            Updated missile position
        """

        # Missile is frozen once it has hit or missed
        if not self.active:
            return self.position

        # --------------------------------------------------
        # Motor phase tracking
        # --------------------------------------------------
        self._flight_time += dt

        # --------------------------------------------------
        # BOOST PHASE
        # High-thrust axial acceleration from dead stop.
        # PN guidance is inactive - missile climbs vertically.
        # Speed ramps linearly from 0 to MISSILE_SPEED over BOOST_TIME.
        # Ref: MIL-HDBK-1211(MI)
        # Ref: Jane's Land-Based Air Defence - SAM boost profiles
        # --------------------------------------------------
        if self._flight_time <= config.BOOST_TIME:
            self.speed = min(
                self.speed + config.BOOST_ACCEL * dt,
                config.MISSILE_SPEED
            )
            self.velocity = np.array([0.0, 0.0, 1.0]) * self.speed
            self.position = self.position + self.velocity * dt
            return self.position

        # --------------------------------------------------
        # SUSTAIN PHASE - PN guidance active from here down
        # Lock speed to MISSILE_SPEED for sustain normalization
        # --------------------------------------------------
        self.speed = config.MISSILE_SPEED

        # --------------------------------------------------
        # Check intercept condition
        # Ref: MIL-HDBK-1211(MI)
        # --------------------------------------------------
        r = target_pos - self.position
        distance = np.linalg.norm(r)

        if distance < config.KILL_DISTANCE:
            print(f"  [HIT] distance = {distance:.2f} m")
            self.state = MissileState.HIT
            return self.position
        
        # --------------------------------------------------
        # MISSED condition
        # If closing velocity goes negative the missile is
        # moving away from the target - it has passed through
        # or flown past. No recovery possible.
        # Ref: MIL-HDBK-1211(MI)
        # --------------------------------------------------
        closing_vel = -np.dot(r, target_vel - self.velocity) / np.linalg.norm(r)
        if closing_vel < 0:
            print(f"  [MISS] closing velocity negative at t, miss distance = {distance:.1f} m")
            self.state = MissileState.MISSED
            return self.position

        # --------------------------------------------------
        # Proportional Navigation guidance
        # Ref: DTIC ADP010953
        # --------------------------------------------------
        accel = proportional_navigation(
            self.position,
            self.velocity,
            target_pos,
            target_vel,
            N=config.NAV_CONSTANT
        )

        # --------------------------------------------------
        # Lateral g-force limiting
        # Only lateral (perpendicular to velocity) acceleration
        # is limited - axial thrust is separate.
        # Ref: MIL-HDBK-1211(MI)
        # --------------------------------------------------
        vel_hat = self.velocity / np.linalg.norm(self.velocity)
        accel_lateral = accel - np.dot(accel, vel_hat) * vel_hat
        lateral_mag   = np.linalg.norm(accel_lateral)

        if lateral_mag > config.MAX_ACCEL:
            accel_lateral = accel_lateral * (config.MAX_ACCEL / lateral_mag)

        # Track peak g-load (lateral only)
        # Ref: MIL-HDBK-1211(MI)
        current_g = lateral_mag / config.GRAVITY
        if current_g > self.peak_g:
            self.peak_g = current_g

        # --------------------------------------------------
        # Update velocity: apply lateral acceleration only
        # Speed is held constant by propulsion (constant-speed
        # assumption for boost-sustain motors)
        # Ref: MIL-HDBK-1211(MI)
        # --------------------------------------------------
        self.velocity = self.velocity + accel_lateral * dt

        # --------------------------------------------------
        # Gravity
        # Applied after lateral acceleration but before
        # speed normalization. The sustain motor re-normalization
        # below counteracts gravity's effect on speed, but
        # gravity's directional pull (nose-down) is preserved
        # in the updated velocity direction.
        # Ref: MIL-HDBK-1211(MI)
        # Ref: U.S. Standard Atmosphere 1976, Table 2
        # --------------------------------------------------
        self.velocity += np.array([0.0, 0.0, -config.GRAVITY]) * dt

        # Re-normalize to maintain constant speed
        speed = np.linalg.norm(self.velocity)
        if speed > 0:
            self.velocity = (self.velocity / speed) * self.speed

        # --------------------------------------------------
        # Update position: x = x + v*dt
        # --------------------------------------------------
        self.position = self.position + self.velocity * dt

        return self.position