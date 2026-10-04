"""Vehicle, tower and scenario parameters.

Numbers are public estimates, not SpaceX data:
  * Raptor 2 sea-level thrust 2,256 kN, throttle 40-100%   (Wikipedia, "SpaceX Raptor")
  * Super Heavy empty mass ~275 t                            (Wikipedia, "SpaceX Super Heavy")
  * landing burn starts on the 13 inner engines, then the 3 centre engines
    finish the catch (flight 5 onwards; one flight finished on 2)
  * sea-level Isp ~327 s, diameter 9 m, height ~71 m
Everything uncertain is a named field so it can be varied in the Monte Carlo.

Frame: x east (towards the tower from offshore), y north, z up; the tower base is the
origin and the catch target is the booster's centre of mass position at capture.
"""
from dataclasses import dataclass, field, replace

import numpy as np

G0 = 9.80665


@dataclass(frozen=True)
class Vehicle:
    engine_thrust: float = 2.256e6     # N, one Raptor 2 at sea level
    throttle_min: float = 0.40
    throttle_max: float = 1.00
    isp: float = 327.0                 # s
    dry_mass: float = 275e3            # kg
    landing_propellant: float = 45e3   # kg available for the landing burn (estimate)
    diameter: float = 9.0              # m
    length: float = 71.0               # m
    cd: float = 0.9                    # axial drag coefficient, tail first (estimate)
    cd_side: float = 0.7               # crossflow drag coefficient of the side (estimate)
    max_tilt_deg: float = 15.0         # max thrust tilt from vertical the guidance uses
    divert_tilt_deg: float = 30.0      # physical limit, used only to divert away from the tower
    final_tilt_deg: float = 3.0        # thrust must be near vertical in the final seconds

    @property
    def wet_mass(self):
        return self.dry_mass + self.landing_propellant

    @property
    def alpha(self):
        """kg of propellant per N*s of thrust."""
        return 1.0 / (self.isp * G0)

    @property
    def area(self):
        return np.pi * (self.diameter / 2) ** 2

    @property
    def side_area(self):
        return self.diameter * self.length

    def thrust_bounds(self, n_engines):
        return (n_engines * self.engine_thrust * self.throttle_min,
                n_engines * self.engine_thrust * self.throttle_max)


@dataclass(frozen=True)
class Tower:
    catch_point: tuple = (0.0, 0.0, 65.0)  # booster centre of mass at capture (estimate)
    # the booster stays offshore of the plane x = -standoff until `commit_time` seconds
    # before the terminal gate, then translates in over the launch mount
    standoff: float = 15.0
    commit_time: float = 6.0
    approach_speed_max: float = 10.0        # m/s towards the tower inside the commit window
    # Terminal descent. Guidance delivers the booster to a gate `gate_height` above the
    # catch point, centred and moving straight down; from there a gentle constant
    # deceleration brings it through the arms at `final_descent_rate`. Decelerating (not
    # coasting at constant speed) keeps thrust above the three engines' minimum, which is
    # only ~2% below the near-empty booster's weight: a constant-speed descent would sit on
    # the throttle floor with no way to correct downwards.
    gate_height: float = 12.0               # m above the catch point
    terminal_decel: float = 1.0             # m/s^2
    final_descent_rate: float = 1.0         # m/s downwards through the arms
    terminal_tilt_deg: float = 2.7          # thrust tilt limit in the terminal descent
    # capture tolerances (arms open ~ +-1 m around the booster's catch pins)
    tol_lateral: float = 1.0
    tol_vertical_speed: float = 1.5
    tol_lateral_speed: float = 0.5
    tol_tilt_deg: float = 3.0

    @property
    def gate_speed(self):
        return float(np.sqrt(self.final_descent_rate ** 2 + 2 * self.terminal_decel * self.gate_height))

    @property
    def terminal_duration(self):
        return (self.gate_speed - self.final_descent_rate) / self.terminal_decel

    @property
    def gate_point(self):
        return np.array(self.catch_point) + np.array([0.0, 0.0, self.gate_height])


@dataclass(frozen=True)
class Scenario:
    """State at landing-burn ignition (after boostback, coasting back tail first)."""
    r0: tuple = (-280.0, 60.0, 1250.0)      # m
    v0: tuple = (30.0, -5.0, -270.0)        # m/s
    engines_phase1: int = 13
    engines_phase2: int = 3
    switch_time: float = 3.0                # s after ignition: 13 -> 3 engines (a later switch
                                            # would need 13 engines below minimum throttle)

    def with_(self, **kw):
        return replace(self, **kw)


SUPER_HEAVY = Vehicle()
TOWER = Tower()
NOMINAL = Scenario()
