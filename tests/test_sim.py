"""Simulator and controller checks."""
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

from boostercatch import controllers as C, sim  # noqa: E402
from boostercatch.vehicle import G0, NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402


class ConstantThrust:
    def __init__(self, T):
        self.T = T

    def reset(self, *a):
        pass

    def command(self, t, r, v, n):
        return np.array([0.0, 0.0, self.T])


def test_integrator_matches_rocket_equation():
    """Constant vertical thrust, no drag, no lag: compare with the exact solution
    v(t) = v0 - g t + ve ln(m0/m),  z(t) = z0 + v0 t - g t^2/2 + ve (t - m/mdot ln(m0/m))."""
    T = 13 * SUPER_HEAVY.engine_thrust * 0.6
    disp = sim.Dispersion(cd_scale=0.0, tau_lag=0.0)
    res = sim.simulate(ConstantThrust(T), SUPER_HEAVY, TOWER, NOMINAL, disp, t_max=2.5, record_every=1)
    ve = SUPER_HEAVY.isp * G0
    mdot = T / ve
    m0, z0, v0 = SUPER_HEAVY.wet_mass, NOMINAL.r0[2], NOMINAL.v0[2]
    i = int(np.searchsorted(res.t, 2.0))
    t = res.t[i]
    m = m0 - mdot * t
    v_exact = v0 - G0 * t + ve * np.log(m0 / m)
    z_exact = z0 + v0 * t - 0.5 * G0 * t ** 2 + ve * (t - m / mdot * np.log(m0 / m))
    assert res.m[i] == pytest.approx(m, rel=1e-6)
    assert res.v[i][2] == pytest.approx(v_exact, abs=1e-3)
    assert res.r[i][2] == pytest.approx(z_exact, abs=1e-3)


def test_ballistic_impact_and_tower_geometry():
    r, v = np.array([-50.0, 0.0, sim.GROUND_Z + 122.5]), np.array([10.0, 0.0, 0.0])
    # falls 122.5 m in 5 s (g ~ 9.8): moves 50 m sideways
    imp = sim.ballistic_impact(r, v)
    assert imp[0] == pytest.approx(-50 + 10 * np.sqrt(2 * 122.5 / G0), rel=1e-9)
    assert sim.tower_hit(np.array([5.0, 0.0, 80.0]))          # skin 0.5 m into the tower face
    assert not sim.tower_hit(np.array([4.0, 0.0, 80.0]))      # 0.5 m clear
    assert not sim.tower_hit(np.array([10.0, 0.0, 200.0]))    # above the tower


def test_estimator_recovers_thrust_effectiveness():
    """A 4% thrust shortfall should show up as k_hat ~ 0.96 within a few seconds."""
    ctl = C.Baseline()
    res = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, sim.Dispersion(thrust_bias=-0.04), t_max=8.0)
    assert ctl.k_hat == pytest.approx(0.96, abs=0.01)
    assert np.linalg.norm(ctl.d_hat) < 0.3


@pytest.mark.parametrize("ctl", [C.Baseline(), C.ConvexMPC()], ids=["baseline", "mpc"])
def test_nominal_flight_is_caught(ctl):
    res = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL)
    assert res.caught, res.reason
    assert res.capture["lateral_error"] < 0.5
    assert abs(res.capture["vz"]) < 1.3
    assert res.capture["propellant_left"] > 1500


def test_late_engine_failure_diverts_clear_of_the_pad():
    """An engine lost 6 s into the burn leaves too little braking for a catch: the
    controller must abort and come down outside the pad zone (or still catch)."""
    res = sim.simulate(C.Baseline(), SUPER_HEAVY, TOWER, NOMINAL, sim.Dispersion(engine_out_time=6.0))
    assert res.caught or (res.pad_safe and res.abort_reason is not None)
