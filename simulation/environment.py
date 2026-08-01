import numpy as np
from dataclasses import dataclass
import config
from simulation.target import Target
from simulation.missile import Missile
from simulation.missile import MissileState


@dataclass
class SimResult:
    intercepted: bool
    intercept_time: float
    final_miss_distance: float
    peak_g: float


class Environment:
    """
    Runs the simulation loop, coupling Target and Missile each timestep.

    Target velocity is estimated by finite difference between consecutive
    position samples - this is consistent with how a real seeker would
    estimate target velocity from successive measurements.

    The missile is held at its start position until MISSILE_LAUNCH_TIME.
    """

    def __init__(self):
        self.target = Target()
        self.missile = Missile()
        self.times = np.arange(0, config.TMAX, config.DT)
        self.target_states = []
        self.missile_states = []

    def run(self):
        print("Running simulation...")

        prev_target_pos  = None
        intercepted      = False
        intercept_time   = None
        prev_peak_g      = 0.0        # for spike detection
        DEBUG_INTERVAL   = 500        # print every 500 steps = every 0.5s
        G_SPIKE_THRESH   = 200.0      # print immediately if g exceeds this

        for t in self.times:
            target_pos = self.target.position(t, self.missile.position)

            # Estimate target velocity by finite difference
            if prev_target_pos is not None:
                target_vel = (target_pos - prev_target_pos) / config.DT
            else:
                target_vel = np.zeros(3)
            prev_target_pos = target_pos.copy()

            # --------------------------------------------------
            # Hold missile at start position until launch time
            # --------------------------------------------------
            if t < config.MISSILE_LAUNCH_TIME:
                self.target_states.append(target_pos)
                self.missile_states.append(self.missile.position.copy())
                continue

            missile_pos = self.missile.step(target_pos, target_vel, config.DT)

            if not self.missile.active and not intercepted:
                intercepted    = self.missile.state == MissileState.HIT
                intercept_time = t if intercepted else None

            self.target_states.append(target_pos)
            self.missile_states.append(missile_pos)

        self.target_states  = np.array(self.target_states)
        self.missile_states = np.array(self.missile_states)

        final_miss = np.linalg.norm(
            self.target_states[-1] - self.missile_states[-1]
        )

        result = SimResult(
            intercepted=intercepted,
            intercept_time=intercept_time if intercept_time is not None else float('nan'),
            final_miss_distance=final_miss,
            peak_g=self.missile.peak_g
        )

        self._print_summary(result)
        return self.target_states, self.missile_states, result

    def _print_summary(self, result):
        print("=" * 52)
        print("  SIMULATION SUMMARY")
        print("=" * 52)
        print(f"  Missile launch time : {config.MISSILE_LAUNCH_TIME:.2f} s")
        print(f"  Target speed        : {config.TARGET_SPEED} m/s")
        print(f"  Missile speed       : {config.MISSILE_SPEED} m/s")
        print(f"  Nav constant (N)    : {config.NAV_CONSTANT}")
        print(f"  Peak lateral g      : {result.peak_g:.1f} g")
        if result.intercepted:
            print(f"  Result              : INTERCEPT at t={result.intercept_time:.2f}s")
        else:
            print(f"  Result              : MISS")
            print(f"  Final miss distance : {result.final_miss_distance:.1f} m")
        print("=" * 52)