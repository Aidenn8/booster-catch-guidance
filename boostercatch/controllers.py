"""Two ways to fly the landing burn, given the same sensors, engines and terminal descent.

ConvexMPC     plans a fuel-optimal trajectory to the terminal gate (lossless
              convexification), re-plans every second from the current state, and
              between re-plans tracks the plan with a PD correction. Re-plans immediately
              after an engine failure, with the new engine count.

Baseline      an engineered heuristic in the spirit of simpler public catch simulators:
              during the 13-engine phase it brakes just enough to arrive at the engine
              switch on a constant-deceleration speed profile, then the 3 engines follow
              that profile down to the gate; sideways it flies a braking profile
              towards a point offshore of the tower, and at the commit window towards
              the arms.

Shared by both (so the comparison isolates the guidance):
  * a thrust model (commanded thrust, clipped to the nominal engine limits, through the
    nominal throttle lag) used to estimate mass; neither controller knows the true mass
  * an observer that splits what thrust, gravity and the nominal drag model do not
    explain into (a) a thrust-effectiveness factor k (unexplained acceleration along the
    thrust: thrust bias and mass error, which scale with thrust) and (b) a residual
    acceleration across it (mostly wind); commands are scaled by 1/k and the residual is
    fed forward with the drag model
  * the terminal descent law from the gate through the arms
"""
from dataclasses import replace

import numpy as np

from . import guidance
from .vehicle import G0

GRAV = np.array([0.0, 0.0, -G0])
TAU_LAG = 0.25        # s, nominal throttle response (the sim's truth uses the same value)


def limit_tilt(u, max_deg):
    """Fit a thrust-acceleration command into the tilt cone (half-angle max_deg about
    vertical) by trimming its horizontal part and keeping the vertical part. Rotating the
    whole vector instead would turn an oversized sideways request into extra braking,
    which near the arms stalls the booster (it cannot then descend: at minimum throttle
    the three engines roughly balance its weight)."""
    u = np.asarray(u, float)
    if u[2] <= 0:
        return np.array([0.0, 0.0, 1e-6])
    h = np.linalg.norm(u[:2])
    h_max = u[2] * np.tan(np.radians(max_deg))
    if h <= h_max:
        return u
    return np.array([u[0] * h_max / h, u[1] * h_max / h, u[2]])


class _Base:
    obs_omega, obs_zeta = 1.5, 0.8          # disturbance observer bandwidth (rad/s), damping
    term_kv, term_kp, term_kd = 1.5, 0.6, 1.5

    def reset(self, veh, tower, scen):
        self.veh, self.tower, self.scen = veh, tower, scen
        self.m_est = veh.wet_mass
        self.last_t = None
        self.T_model = np.zeros(3)
        self.engine_out = False
        self.v_hat = np.array(scen.v0, float)
        self.d_hat = np.zeros(3)
        self.k_hat = 1.0
        self.in_terminal = False
        self.abort_reason, self.abort_time = None, None

    def notify_engine_out(self, t):
        self.engine_out = True

    def abort(self, t, reason):
        """Give up on the catch and divert: a booster that cannot be caught must come down
        clear of the tower and the launch mount (in reality: a splashdown offshore)."""
        if self.abort_reason is None:
            self.abort_reason, self.abort_time = reason, t

    # -- estimation -------------------------------------------------------------
    def drag_model(self, v):
        """Nominal drag (still air, nominal coefficients) at the estimated mass."""
        return guidance.drag_accel(v, self.m_est, np.zeros(3), self.veh)

    def accel_ff(self, v):
        """Feed-forward for everything except thrust and gravity: drag model + residual."""
        return self.drag_model(v) + self.d_hat

    def _estimate(self, t, v_meas):
        if self.last_t is None:
            self.last_t = t
            self.v_hat = v_meas.copy()
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0:
            return
        self.m_est -= np.linalg.norm(self.T_model) * dt / (self.veh.isp * G0)
        # the drag model is inside the observer's prediction, so the estimates only have to
        # track slowly varying errors; observing total drag would lag badly while drag falls
        # from ~8 to ~1 m/s^2 in the first seconds of the burn
        a_T = self.T_model / self.m_est
        a_model = self.k_hat * a_T + GRAV + self.drag_model(self.v_hat)
        e = v_meas - self.v_hat
        self.v_hat += (a_model + self.d_hat + 2 * self.obs_zeta * self.obs_omega * e) * dt
        nT = np.linalg.norm(a_T)
        if nT > 1.0:
            eT = a_T / nT
            e_par = float(e @ eT)
            self.k_hat = float(np.clip(self.k_hat + self.obs_omega ** 2 * e_par / nT * dt, 0.85, 1.15))
            self.d_hat += self.obs_omega ** 2 * (e - e_par * eT) * dt
        else:
            self.d_hat += self.obs_omega ** 2 * e * dt

    @property
    def m_eff(self):
        """Mass to multiply a desired thrust acceleration by: estimated mass divided by the
        estimated thrust effectiveness."""
        return self.m_est / self.k_hat

    def _output(self, cmd, n_eng, dt=0.02):
        """Advance the thrust model: what the engines should be doing given this command."""
        lo, hi = self.veh.thrust_bounds(n_eng)
        mag = np.linalg.norm(cmd)
        direc = cmd / mag if mag > 0 else np.array([0, 0, 1.0])   # already inside the tilt cone
        applied = direc * np.clip(mag, lo, hi)
        self.T_model += (applied - self.T_model) * min(1.0, dt / TAU_LAG)
        return cmd

    # -- terminal descent (shared) ------------------------------------------------
    def terminal_command(self, r, v):
        """From the gate: follow v(h) = -sqrt(vf^2 + 2*a*h) (constant deceleration a down
        to vf at the arms), centre over the catch point with a tilt-limited correction,
        and cancel the estimated disturbance."""
        tw = self.tower
        target = np.array(tw.catch_point)
        h = r[2] - target[2]
        v_des = -np.sqrt(tw.final_descent_rate ** 2 + 2 * tw.terminal_decel * max(h, 0.0))
        az = G0 + tw.terminal_decel + self.term_kv * (v_des - v[2])
        a_h = self.term_kp * (target[:2] - r[:2]) - self.term_kd * v[:2]
        u = np.array([a_h[0], a_h[1], az]) - self.accel_ff(v)
        return self.m_eff * limit_tilt(u, tw.terminal_tilt_deg)

    def _stop_point(self, r, v):
        """Where the booster would stop sideways if it braked at the terminal descent's
        maximum sideways acceleration (thrust ~ weight, tilted terminal_tilt_deg)."""
        a_max = (G0 + self.tower.terminal_decel) * np.tan(np.radians(self.tower.terminal_tilt_deg))
        vh = v[:2]
        return r[:2] + vh * np.linalg.norm(vh) / (2 * a_max)

    def gate_check(self, r, v):
        """Go/no-go at the gate: the predicted sideways stopping point must be inside the
        arms' tolerance (plus half a metre the descent has time to remove)."""
        target = np.array(self.tower.catch_point)
        return np.linalg.norm(self._stop_point(r, v) - target[:2]) <= self.tower.tol_lateral + 0.5

    def keep_out_check(self, r, v):
        """During the terminal descent, the predicted stopping point must keep the booster's
        skin at least 1 m from the tower's front face (x = 9 m)."""
        return self._stop_point(r, v)[0] + 0.5 * self.veh.diameter < 9.0 - 1.0

    def engines_for_planning(self, t):
        e1, e2 = self.scen.engines_phase1, self.scen.engines_phase2
        if self.engine_out:
            if t < self.scen.switch_time:
                e1 -= 1          # the failed engine is lost for the rest of phase 1...
            e2 -= 1              # ...and we assume (conservatively) it is one of the centre three
        return e1, e2

    DIVERT_X = -90.0        # m: divert aim point, offshore of the launch mount
    DIVERT_DESCENT = 8.0    # m/s: descent rate held during a divert

    def divert_command(self, r, v, n_eng):
        """Steer towards a point offshore (full tilt allowed) while holding a moderate
        descent rate, braking as hard as needed."""
        target = np.array(self.tower.catch_point)
        # always aim further offshore than the booster is now: a fixed aim point would pull
        # a booster that is already beyond it back towards the tower
        aim = np.array([min(target[0] + self.DIVERT_X, r[0] - 30.0), r[1]])
        # stiff gains: the sideways request must saturate the tilt limit while the booster is
        # still drifting towards the tower. With soft gains the (huge) braking request
        # dominates, and once the total is clipped to maximum thrust the tilt stays near 20 deg.
        a_h = 1.0 * (aim - r[:2]) - 2.5 * v[:2]
        az = G0 + 1.5 * (-self.DIVERT_DESCENT - v[2]) - self.accel_ff(v)[2]
        return self.m_eff * limit_tilt(np.array([a_h[0], a_h[1], max(az, 1.0)]), self.veh.divert_tilt_deg)

    def command(self, t, r, v, n_eng):
        self._estimate(t, v)
        h = r[2] - self.tower.catch_point[2]
        if self.abort_reason is not None:
            return self._output(self.divert_command(r, v, n_eng), n_eng)
        if not self.in_terminal and t > self.scen.switch_time and self._enter_terminal(t, h, v[2]):
            self.in_terminal = True
            if not self.gate_check(r, v):
                self.abort(t, "no-go at the gate")
                return self._output(self.divert_command(r, v, n_eng), n_eng)
        if self.in_terminal:
            if not self.keep_out_check(r, v):
                self.abort(t, "tower keep-out")
                return self._output(self.divert_command(r, v, n_eng), n_eng)
            cmd = self.terminal_command(r, v)
        else:
            cmd = self.approach_command(t, r, v, n_eng)
            if self.abort_reason is not None:     # the approach logic just gave up
                return self._output(self.divert_command(r, v, n_eng), n_eng)
            cmd = limit_tilt(cmd, self.veh.max_tilt_deg)
        return self._output(cmd, n_eng)

    def _enter_terminal(self, t, h, vz):
        """Switch to the terminal descent one throttle lag before reaching the gate: the
        engines need ~TAU_LAG to drop from approach braking to the terminal thrust, and
        braking too long there leaves the booster crawling above the arms (it cannot speed
        its descent back up against the throttle floor)."""
        return h <= self.tower.gate_height + max(-vz, 0.0) * TAU_LAG


class ConvexMPC(_Base):
    name = "Convex MPC"

    shifts = (0.0, 0.5, 1.0, 2.0, 3.0, 4.0, -0.5)   # gate-time changes tried at a re-plan (s)
    abort_horizon = 3.0   # s before the gate: a failed re-plan earlier than this triggers a divert
    recovery_gate_speed = 12.0   # m/s: fastest gate arrival the recovery plan may use
    residual_cap = 1.0           # m/s^2 per axis: cap on the disturbance estimate fed to the planner

    def __init__(self, replan_every=1.0, first_replan=0.5, freeze_last=2.5, margin=0.08, extra_time=0.0,
                 kp=0.6, kd=1.6, n_nodes=32, approach_cap=False, tilt_margin=2.0, eo_reserve=False):
        self.replan_every, self.first_replan, self.freeze_last = replan_every, first_replan, freeze_last
        self.margin, self.extra_time, self.kp, self.kd, self.n_nodes = margin, extra_time, kp, kd, n_nodes
        self.approach_cap, self.tilt_margin, self.eo_reserve = approach_cap, tilt_margin, eo_reserve

    def reset(self, veh, tower, scen):
        super().reset(veh, tower, scen)
        self.plan, self.plan_t0, self.t_gate = None, 0.0, None
        self.next_replan, self.n_solves, self.failed_replans = 0.0, 0, 0
        self.force_replan = False
        self.fail_times = []
        self.recoveries = 0

    def notify_engine_out(self, t):
        if not self.engine_out:
            self.force_replan = True
        super().notify_engine_out(t)

    def _enter_terminal(self, t, h, vz):
        return super()._enter_terminal(t, h, vz) or (self.t_gate is not None and t >= self.t_gate)

    def _residual(self):
        """Acceleration the drag model and the thrust-effectiveness estimate do not explain
        (mostly wind), held constant over the plan (offset-free MPC); capped, since it is
        an estimate."""
        return np.clip(self.d_hat, -self.residual_cap, self.residual_cap)

    def planning_vehicle(self):
        """The vehicle as the planner should see it: engine thrust scaled by the estimated
        effectiveness. (Scaling the mass instead would be equivalent for accelerations but
        would corrupt the propellant budget, which the planner reads off the mass.)"""
        return replace(self.veh, engine_thrust=self.veh.engine_thrust * self.k_hat)

    def _tilt_margin(self, margin):
        """Tilt reserve that goes with a thrust margin (2 deg at the default 8%)."""
        return self.tilt_margin * (margin / self.margin) if self.margin > 0 else 0.0

    def _solve(self, t, r, v, tf, margin, gate_speed_max=None):
        e = self.engines_for_planning(t)
        if self.engine_out and t >= self.scen.switch_time:
            e = (e[1], e[1])
        v_guess = None
        if self.plan is not None:   # drag on the previous plan's velocities (then the gate velocity)
            p, t0 = self.plan, self.plan_t0
            v_end = np.array([0.0, 0.0, -self.tower.gate_speed])

            def v_guess(x):
                tau = t + x - t0
                if tau >= p.t[-1]:
                    return v_end
                return np.array([np.interp(tau, p.t, p.v[:, i]) for i in range(3)])
        return guidance.solve_with_drag(
            self.planning_vehicle(), self.tower, r, v, self.m_est, tf, passes=1, n_nodes=self.n_nodes,
            v_guess=v_guess,
            switch_time=self.scen.switch_time, engines=e, elapsed=t, total_tf=t + tf, margin=margin,
            extra_accel=self._residual(), gate_speed_max=gate_speed_max, approach_cap=self.approach_cap,
            tilt_margin_deg=self._tilt_margin(margin), switch_lag=TAU_LAG,
            eo_reserve=self.eo_reserve and not self.engine_out)

    def _replan(self, t, r, v):
        self.n_solves += 1
        if self.plan is None:
            p = guidance.solve_earliest(
                self.planning_vehicle(), self.tower, r, v, self.m_est, extra=self.extra_time, n_nodes=self.n_nodes,
                switch_time=self.scen.switch_time, engines=self.engines_for_planning(t), margin=self.margin,
                approach_cap=self.approach_cap, tilt_margin_deg=self.tilt_margin, switch_lag=TAU_LAG,
                eo_reserve=self.eo_reserve and not self.engine_out)
            if p is not None:
                self.plan, self.plan_t0, self.t_gate = p, t, t + p.tf
            else:
                self.abort(t, "no feasible trajectory")
            return
        remaining = self.t_gate - t
        # Prefer keeping the gate time, then later ones (always feasible if the booster can
        # still brake in time), and an earlier one last: an earlier gate saves a little fuel
        # but leaves less slack. Keep the thrust margin if possible; if the booster has
        # drifted to where no plan fits inside it, use the full thrust range rather than
        # fly a stale plan.
        # A plan that sits exactly on its margin can become infeasible at the next re-plan
        # from re-gridding noise alone; slipping the gate costs ~0.5 t per half second, so
        # first try keeping the gate time with three quarters of the margin.
        tries = [(0.0, self.margin), (0.0, 0.75 * self.margin)]
        tries += [(d, m) for m in (self.margin, self.margin / 2, 0.0) for d in self.shifts if (d, m) not in tries]
        for d, margin in tries:
            if remaining + d < 1.0:
                continue
            p = self._solve(t, r, v, remaining + d, margin)
            if p is not None:
                self.plan, self.plan_t0, self.t_gate = p, t, t + p.tf
                return
        self.failed_replans += 1
        self.fail_times.append((round(t, 2), round(remaining, 2)))
        # Recovery: reach the gate centred but allow a faster descent through it (the
        # terminal descent has the authority to absorb it), full thrust range.
        for d in self.shifts:
            if remaining + d < 1.0:
                continue
            p = self._solve(t, r, v, remaining + d, 0.0, gate_speed_max=self.recovery_gate_speed)
            if p is not None:
                self.plan, self.plan_t0, self.t_gate = p, t, t + p.tf
                self.recoveries += 1
                return
        # No trajectory to the gate exists from here even then: under the model, the
        # convex solver's infeasibility certifies that the catch is lost, so divert now
        # rather than keep chasing a stale plan towards the tower.
        if remaining > self.abort_horizon:
            self.abort(t, "no feasible trajectory")

    def approach_command(self, t, r, v, n_eng):
        remaining = None if self.t_gate is None else self.t_gate - t
        if self.force_replan or (t >= self.next_replan and (remaining is None or remaining > self.freeze_last)):
            self._replan(t, r, v)
            # the first re-plan comes early (it absorbs the engines' start-up transient);
            # with no plan yet, retry twice a second rather than every simulation step
            gap = self.first_replan if self.n_solves == 1 or self.plan is None else self.replan_every
            self.next_replan = t + gap
            self.force_replan = False
        if self.plan is None:  # no feasible plan: brake as hard as possible, straight up
            return np.array([0, 0, 1.0]) * n_eng * self.veh.engine_thrust
        p = self.plan
        tau = np.clip(t - self.plan_t0, 0.0, p.t[-1])
        r_ref = np.array([np.interp(tau, p.t, p.r[:, i]) for i in range(3)])
        v_ref = np.array([np.interp(tau, p.t, p.v[:, i]) for i in range(3)])
        # lead compensation: the engines respond with a ~TAU_LAG first-order lag, and the plan
        # switches thrust abruptly (max-min-max); commanding the plan TAU_LAG early delivers
        # the planned impulse instead of losing ~TAU_LAG x (step) at every switch
        k = int(np.clip(np.searchsorted(p.t, tau + TAU_LAG, side="right") - 1, 0, len(p.accel) - 1))
        u = p.accel[k] + self.kp * (r_ref - r) + self.kd * (v_ref - v)
        return self.m_eff * u


class Baseline(_Base):
    name = "Baseline"

    def __init__(self, a2=8.0, kv=1.2, a_lat=3.0, k_pos=0.8, k_vel=1.5, v_lat_max=45.0):
        self.a2_nominal, self.kv = a2, kv
        self.a_lat, self.k_pos, self.k_vel, self.v_lat_max = a_lat, k_pos, k_vel, v_lat_max

    def reset(self, veh, tower, scen):
        super().reset(veh, tower, scen)
        self.a2 = self.a2_nominal

    def notify_engine_out(self, t):
        """Keep the same fraction of the remaining braking capability (net deceleration
        with the phase-2 engines at full throttle, at the estimated mass)."""
        if not self.engine_out:
            n2 = self.scen.engines_phase2
            T = self.veh.engine_thrust * self.veh.throttle_max
            net = lambda n: n * T / self.m_eff - G0
            self.a2 = self.a2_nominal * max(net(n2 - 1), 0.5) / net(n2)
        super().notify_engine_out(t)

    def v_ref(self, h):
        """Constant-deceleration speed profile reaching the gate speed at the gate height."""
        tw = self.tower
        return -np.sqrt(tw.gate_speed ** 2 + 2 * self.a2 * max(h - tw.gate_height, 0.0))

    def approach_command(self, t, r, v, n_eng):
        tw = self.tower
        target = np.array(tw.catch_point)
        h = r[2] - target[2]
        d = self.accel_ff(v)
        if t < self.scen.switch_time:
            # constant vertical acceleration for the rest of phase 1 that lands the state on
            # the phase-2 speed profile exactly at the engine switch (bisection)
            tau = self.scen.switch_time - t
            lo, hi = -50.0, 200.0
            for _ in range(40):
                a = 0.5 * (lo + hi)
                v1 = v[2] + a * tau
                h1 = h + v[2] * tau + 0.5 * a * tau ** 2
                if v1 - self.v_ref(h1) < 0:   # still too fast downwards: brake harder
                    lo = a
                else:
                    hi = a
            az = 0.5 * (lo + hi) + G0 - d[2]
            v_sw = self.v_ref(max(h + v[2] * tau, 0.0))
            t_gate = tau + (abs(v_sw) - tw.gate_speed) / self.a2
        else:
            # braking needed to reach the gate speed at the gate, against what the live
            # engines can give: if it is out of reach, the catch is lost, so divert
            room = h - tw.gate_height
            if room > 5.0 and -v[2] > tw.gate_speed:
                need = (v[2] ** 2 - tw.gate_speed ** 2) / (2 * room)
                n_live = self.scen.engines_phase2 - (1 if self.engine_out else 0)
                avail = n_live * self.veh.engine_thrust * self.veh.throttle_max / self.m_eff - G0 + d[2]
                if need > 0.95 * avail:
                    self.abort(t, "braking out of reach")
            az = G0 + self.a2 + self.kv * (self.v_ref(h) - v[2]) - d[2]
            t_gate = (abs(self.v_ref(h)) - tw.gate_speed) / self.a2
        aim = target.copy()
        if t_gate > tw.commit_time:
            aim[0] = target[0] - tw.standoff - 5.0   # wait offshore until the commit window
        # sideways: fly towards the aim point on a braking profile (constant deceleration
        # a_lat to arrive at rest, capped speed), tracked with a velocity loop
        e = aim[:2] - r[:2]
        dist = np.linalg.norm(e)
        speed = min(np.sqrt(2 * self.a_lat * dist), self.k_pos * dist, self.v_lat_max)
        v_des = e / dist * speed if dist > 1e-6 else np.zeros(2)
        a_lat = self.k_vel * (v_des - v[:2]) - d[:2]
        return self.m_eff * np.array([a_lat[0], a_lat[1], az])
