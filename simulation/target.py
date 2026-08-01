import numpy as np
import config


class Target:
    """
    Models a maneuvering aircraft flying a three-segment trajectory,
    with missile-aware evasive maneuver capability.

    Normal flight: three-segment preset path (straight → banked turn → straight)

    Evasion - chained banked S-turns:
    Once the missile is detected (after boost completes), after a reaction
    delay the target executes a series of chained 120° banked arcs, each
    reversing direction from the last. Each arc includes the same vertical
    drift formula as the preset curve segment, giving a realistic banked
    appearance with altitude change through each turn.

    Arc chain positions are pre-computed including accumulated drift so
    there are no position discontinuities at arc boundaries.

    Ref: Shaw, R.L. - Fighter Combat: Tactics and Maneuvering (1985), p.64-71
    Ref: Zarchan - Tactical and Strategic Missile Guidance (2012)
    Ref: trace_and_chase.py - reference arc geometry (read-only)
    """

    def __init__(self):
        self.start = config.AIRCRAFT_START.copy()
        self.vel   = config.TARGET_SPEED

        # Segment initialization flags (preset trajectory)
        self._curve_start     = None
        self._straight2_start = None
        self._center          = None
        self._radius          = None

        # Evasion state
        self._evading         = False
        self._detection_time  = None
        self._evasion_start_t = None
        self._arcs            = []       # list of arc descriptors
        self._arc_end_times   = []       # cumulative end time for each arc

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def position(self, t, missile_pos=None):
        """
        Calculate target position at time t.

        Parameters
        ----------
        t : float
            Current simulation time [s]
        missile_pos : np.ndarray (3,) or None
            Current missile position [m].
        """

        if not self._evading:
            pos = self._preset_position(t)

            # Only check for missile after boost phase completes
            if (missile_pos is not None
                    and t > config.MISSILE_LAUNCH_TIME + config.BOOST_TIME):
                dist = np.linalg.norm(missile_pos - pos)
                if dist < config.TARGET_DETECTION_RANGE:
                    if self._detection_time is None:
                        self._detection_time = t
                        print(f"  [TARGET] Missile detected at"
                              f" {dist/1000:.1f} km, t={t:.2f}s")

                    if t - self._detection_time >= config.TARGET_REACTION_TIME:
                        self._init_evasion(t, pos, missile_pos)

            return pos

        return self._evasion_position(t)

    # ------------------------------------------------------------------
    # Evasion initialization
    # ------------------------------------------------------------------

    def _init_evasion(self, t, pos, missile_pos):
        """
        Pre-compute the full chain of S-turn arcs.

        Each arc's start position is computed from the DRIFTED end
        position of the previous arc - this ensures there are no
        position discontinuities at arc boundaries.

        The drift formula (cosine envelope) matches the preset curve
        segment exactly. At the end of each arc (tc = arc_dur):
            drift = v² * (1 - cos(π)) * CLIMB_RATE_CURVE
                  = v² * 2 * CLIMB_RATE_CURVE
        This accumulated offset is tracked and propagated to each
        subsequent arc's start position.
        """
        self._evading         = True
        self._evasion_start_t = t

        # ----------------------------------------------------------
        # Current velocity direction
        # ----------------------------------------------------------
        t_prev  = max(t - config.DT, 0.0)
        prev    = self._preset_position(t_prev)
        vel     = pos - prev
        speed   = np.linalg.norm(vel)
        forward = vel / speed if speed > 1.0 else np.array([1.0, 0.0, 0.0])

        # ----------------------------------------------------------
        # Break direction - away from missile
        # ----------------------------------------------------------
        world_up = np.array([0.0, 0.0, 1.0])
        right    = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([0.0, 1.0, 0.0])
        right = right / np.linalg.norm(right)

        to_missile = missile_pos - pos
        if np.linalg.norm(to_missile) > 1e-6:
            to_missile = to_missile / np.linalg.norm(to_missile)
        else:
            to_missile = -forward

        break_dir = -right if np.dot(right, to_missile) > 0 else right

        # ----------------------------------------------------------
        # Arc parameters
        # r = v² / (n*g)
        # Ref: MIL-HDBK-1797 - Flying Qualities of Piloted Aircraft
        # ----------------------------------------------------------
        speed_nom = config.TARGET_SPEED
        radius    = speed_nom**2 / (config.TARGET_MAX_G * config.GRAVITY)
        arc_angle = config.EVASION_TURN_ANGLE
        omega     = speed_nom / radius
        arc_dur   = arc_angle / omega

        # ----------------------------------------------------------
        # Drift at the END of one arc (tc = arc_dur)
        #
        # drift(tc) = v² * (1 - cos(π * tc / arc_dur)) * CLIMB_RATE_CURVE
        # At tc = arc_dur: cos(π * arc_dur / arc_dur) = cos(π) = -1
        # So: drift_end = v² * (1 - (-1)) * CLIMB_RATE_CURVE
        #               = v² * 2 * CLIMB_RATE_CURVE
        #
        # This is the total vertical offset accumulated over one arc.
        # We propagate this to the start of the next arc so positions
        # are continuous across arc boundaries.
        #
        # Ref: trace_and_chase.py target_location() curve segment
        # ----------------------------------------------------------
        yz        = config.YZ_ANGLE
        drift_end = speed_nom**2 * 2.0 * config.CLIMB_RATE_CURVE
        drift_dy  = np.cos(yz + np.pi / 2) * drift_end
        drift_dz  = np.sin(yz + np.pi / 2) * drift_end

        # ----------------------------------------------------------
        # Build arc chain
        # ----------------------------------------------------------
        n_arcs        = max(int((config.TMAX - t) / arc_dur) + 2, 6)
        current_pos   = pos.copy()
        current_fwd   = forward.copy()
        current_break = break_dir.copy()
        cumulative_t  = t

        for i in range(n_arcs):
            arc_center = current_pos + current_break * radius

            # Geometric end position (without drift)
            end_theta    = arc_angle
            end_pos_geom = (arc_center
                            - current_break * radius * np.cos(end_theta)
                            + current_fwd   * radius * np.sin(end_theta))

            # Drifted end position - this becomes the next arc's start
            # The drift offset must be added so the next arc center is
            # computed from the correct (drifted) start position.
            end_pos_drifted      = end_pos_geom.copy()
            end_pos_drifted[1]  += drift_dy
            end_pos_drifted[2]  += drift_dz

            # End forward direction
            end_fwd = (current_fwd   * np.cos(end_theta)
                       + current_break * np.sin(end_theta))
            norm    = np.linalg.norm(end_fwd)
            end_fwd = end_fwd / norm if norm > 1e-6 else current_fwd

            self._arcs.append({
                'start_pos': current_pos.copy(),
                'forward':   current_fwd.copy(),
                'break_dir': current_break.copy(),
                'center':    arc_center.copy(),
                'radius':    radius,
                'angle':     arc_angle,
                'omega':     omega,
                'duration':  arc_dur,
            })

            cumulative_t += arc_dur
            self._arc_end_times.append(cumulative_t)

            # Next arc starts at drifted end of this arc
            current_pos   = end_pos_drifted
            current_fwd   = end_fwd
            current_break = -current_break   # reverse for S-turn

        side = 'left' if np.dot(break_dir, right) < 0 else 'right'
        print(f"  [TARGET] S-turn evasion at t={t:.2f}s"
              f" | r={radius/1000:.1f}km"
              f" | {side} break first"
              f" | {n_arcs} arcs planned")

    # ------------------------------------------------------------------
    # Evasion position
    # ------------------------------------------------------------------

    def _evasion_position(self, t):
        """
        Position along the chained S-turn arc sequence.

        Each arc uses the same banked circle geometry and vertical drift
        formula as the preset curve segment. The drift at tc=0 is always
        zero, and the arc center was computed from the drifted end of
        the previous arc, so positions are continuous at boundaries.

        Ref: trace_and_chase.py target_location() curve segment
        """
        # Find which arc we're in
        arc_idx = len(self._arcs) - 1
        for i, end_t in enumerate(self._arc_end_times):
            if t < end_t - 1e-9:
                arc_idx = i
                break

        arc = self._arcs[arc_idx]

        # Time within this arc, clamped to [0, duration]
        arc_start_t = (self._arc_end_times[arc_idx - 1]
                       if arc_idx > 0
                       else self._evasion_start_t)
        tc = np.clip(t - arc_start_t, 0.0, arc['duration'])

        forward   = arc['forward']
        break_dir = arc['break_dir']
        center    = arc['center']
        radius    = arc['radius']
        omega     = arc['omega']
        duration  = arc['duration']

        theta = omega * tc

        # Base arc position - continuous at tc=0 because center was
        # computed from the drifted start position of this arc
        pos = (center
               - break_dir * radius * np.cos(theta)
               + forward   * radius * np.sin(theta))

        # Vertical drift - cosine envelope, zero at tc=0 and tc=arc_dur
        # At tc=0: drift=0 → no discontinuity
        # At tc=arc_dur: drift=drift_end → matches next arc's start
        # Ref: trace_and_chase.py target_location() curve segment
        speed = config.TARGET_SPEED
        drift = (speed**2
                 * (1 - np.cos(np.pi * tc / duration))
                 * config.CLIMB_RATE_CURVE)

        yz = config.YZ_ANGLE
        pos[1] += np.cos(yz + np.pi / 2) * drift
        pos[2] += np.sin(yz + np.pi / 2) * drift

        return pos

    # ------------------------------------------------------------------
    # Preset trajectory
    # ------------------------------------------------------------------

    def _preset_position(self, t):
        """
        Original three-segment preset trajectory.
        Matches trace_and_chase.py reference exactly.
        """
        v = config.TARGET_SPEED

        if t <= config.STRAIGHT_TIME1:
            return np.array([
                self.start[0] + v * t,
                self.start[1],
                self.start[2]
            ])

        elif t <= config.STRAIGHT_TIME1 + config.CURVE_TIME:
            tc = t - config.STRAIGHT_TIME1

            if self._curve_start is None:
                self._curve_start = np.array([
                    self.start[0] + v * config.STRAIGHT_TIME1,
                    self.start[1],
                    self.start[2]
                ])
                self._radius = (v * config.CURVE_TIME) / config.TURN_ANGLE
                self._center = np.array([
                    self._curve_start[0],
                    self._curve_start[1] + self._radius * np.cos(config.YZ_ANGLE),
                    self._curve_start[2] + self._radius * np.sin(config.YZ_ANGLE)
                ])

            angle     = tc * config.TURN_ANGLE / config.CURVE_TIME
            arc_angle = -np.pi / 2 + angle
            drift     = (v**2
                         * (1 - np.cos(np.pi * tc / config.CURVE_TIME))
                         * config.CLIMB_RATE_CURVE)

            return np.array([
                self._center[0] + self._radius * np.cos(arc_angle),
                self._center[1] + self._radius * np.sin(arc_angle) * np.cos(config.YZ_ANGLE)
                             + np.cos(config.YZ_ANGLE + np.pi / 2) * drift,
                self._center[2] + self._radius * np.sin(arc_angle) * np.sin(config.YZ_ANGLE)
                             + np.sin(config.YZ_ANGLE + np.pi / 2) * drift
            ])

        else:
            if self._straight2_start is None:
                self._straight2_start = self._preset_position(
                    config.STRAIGHT_TIME1 + config.CURVE_TIME
                )

            ts = min(t - (config.STRAIGHT_TIME1 + config.CURVE_TIME),
                     config.STRAIGHT_TIME2)

            dx = np.cos(config.TURN_ANGLE)
            dy = np.sin(config.TURN_ANGLE) * np.cos(config.YZ_ANGLE)
            dz = np.sin(config.TURN_ANGLE) * np.sin(config.YZ_ANGLE)

            return np.array([
                self._straight2_start[0] + v * ts * dx,
                self._straight2_start[1] + v * ts * dy,
                self._straight2_start[2] + v * ts * dz
            ])