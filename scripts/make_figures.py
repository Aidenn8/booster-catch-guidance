"""All figures for the report, from the results/ files. Missing inputs are skipped.

    python scripts/make_figures.py

Every figure is drawn at the size the page shows it (100 CSS px per inch; FULL is the width
of the content column, TEXT the width of a text line), so type reads the same size in every
figure. Black background and Inter, to sit on the report page. Lines are labelled directly
where that is clearer than a legend; panel titles are left-aligned; grids are light and
horizontal; colours are the same for each controller in every figure.
"""
import json
import sys
import warnings
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.font_manager as fm  # noqa: E402
import matplotlib.lines as mlines  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.ticker as mticker  # noqa: E402
import numpy as np  # noqa: E402

from boostercatch import ROOT, controllers as C, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
from summarize import outcome, wilson  # noqa: E402

RES = ROOT / "results"
OUT = ROOT / "docs" / "images"
OUT.mkdir(parents=True, exist_ok=True)
for f in sorted((ROOT / "tools" / "fonts").glob("Inter-*.ttf")):
    fm.fontManager.addfont(str(f))

FULL, TEXT = 9.4, 7.6                     # inches = hundreds of CSS px on the page
BG, INK, DIM, FAINT, GRID, AXIS = "#000000", "#f0f0f0", "#a8a8a8", "#6e6e6e", "#1e1e1e", "#4a4a4a"
ANN, NOTE = 9.0, 8.6                      # annotation and small-note sizes (pt)
COL = {"mpc": "#56B4E9", "baseline": "#E69F00"}
NAME = {"mpc": "Convex MPC", "baseline": "Baseline"}
OC_COL = {"caught": "#14A374", "safe miss": "#5c5c5c", "pad strike": "#E2652A"}
OC_NAME = {"caught": "Caught", "safe miss": "Safe miss", "pad strike": "Pad strike"}
SHORT = {"mpc": "Convex MPC", "mpc[approach_cap=1]": "MPC + approach cap", "mpc[eo_reserve=1]": "MPC + engine-out reserve",
         "baseline": "Baseline"}
plt.rcParams.update({
    "figure.dpi": 100, "savefig.dpi": 200, "figure.constrained_layout.use": True,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "font.family": "Inter", "font.size": 9.8, "axes.unicode_minus": True,
    "text.color": INK, "axes.labelcolor": DIM, "axes.labelsize": 9.6, "axes.labelpad": 6,
    "axes.titlesize": 10.3, "axes.titleweight": "medium", "axes.titlelocation": "left", "axes.titlepad": 12,
    "axes.titlecolor": INK, "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "axes.spines.left": False,
    "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "xtick.color": AXIS, "ytick.color": AXIS, "xtick.labelcolor": DIM, "ytick.labelcolor": DIM,
    "xtick.labelsize": 9.2, "ytick.labelsize": 9.2, "xtick.major.size": 3.5, "ytick.major.size": 0,
    "xtick.major.width": 0.8, "xtick.minor.size": 0, "ytick.minor.size": 0, "ytick.major.pad": 6,
    "legend.frameon": False, "legend.fontsize": 9.2, "legend.labelcolor": INK, "legend.handlelength": 1.6,
    "legend.borderaxespad": 0.2, "legend.columnspacing": 1.4,
    "lines.linewidth": 1.7, "lines.markersize": 5, "patch.linewidth": 0, "hatch.color": "#2c2c2c", "hatch.linewidth": 0.7,
})


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)
    print("  wrote", name)


def plain_log(axis, ticks, fmt=None):
    """Log axis with plain labels (1, 10, 100) instead of powers of ten."""
    axis.set_major_locator(mticker.FixedLocator(ticks))
    axis.set_major_formatter(mticker.FuncFormatter(fmt or (lambda v, _: f"{v:g}")))
    axis.set_minor_locator(mticker.NullLocator())


def say(ax, xy, text, color=DIM, dx=0, dy=0, ha="left", va="center", size=ANN, weight="normal", **kw):
    """A direct label at a data point, offset in points."""
    return ax.annotate(text, xy, xytext=(dx, dy), textcoords="offset points", ha=ha, va=va, color=color,
                       fontsize=size, fontweight=weight, **kw)


def hbar_axes(ax):
    """Value axis along x: vertical grid, no y grid, no baseline."""
    ax.grid(axis="y", visible=False)
    ax.grid(axis="x", visible=True)
    ax.tick_params(axis="x", length=0)
    ax.spines["bottom"].set_visible(False)


def load(name):
    f = RES / name
    if not f.exists():
        print("skip (missing)", name)
        return None
    return [json.loads(line) for line in open(f)]


def family(key):
    return key.split("[")[0]


def label(key):
    """'mpc[approach_cap=1]' -> 'Convex MPC + approach cap'."""
    name = NAME[family(key)]
    rest = key[len(family(key)):].strip("[]")
    if not rest:
        return name
    if rest == "approach_cap=1":
        return f"{name} + approach cap"
    if rest == "eo_reserve=1":
        return f"{name} + engine-out reserve"
    rest = rest.replace("approach_cap=0", "no approach cap").replace("margin=", "margin ").replace(",", ", ")
    return f"{name} ({rest})"


def ctl_keys(rows):
    """Controllers in a results file: MPC, its variants, then the baseline."""
    return sorted({d["controller"] for d in rows}, key=lambda k: (family(k) != "mpc", "[" in k, k))


def color(key):
    if "eo_reserve=1" in key:
        return "#DA8FC2"
    c = COL[family(key)]
    return c if "[" not in key else {"mpc": "#A9D6F5", "baseline": "#F0C36B"}[family(key)]


def draw_tower(ax, labels=False):
    ax.add_patch(mpatches.Rectangle((9, 0), 12, 146, color="#2b2b2b", lw=0, zorder=1))
    ax.plot([-4, 9], [TOWER.catch_point[2] + 30] * 2, color="#bdbdbd", lw=1.6, solid_capstyle="butt", zorder=2)
    ax.add_patch(mpatches.Rectangle((-12, 0), 24, 20, color="#1f1f1f", lw=0, zorder=1))   # launch mount


def nominal_runs():
    runs = {}
    for key, ctl in (("mpc", C.ConvexMPC()), ("baseline", C.Baseline())):
        runs[key] = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, record_every=2)
        cap = runs[key].capture
        print(f"nominal {key}: {runs[key].reason}, lateral {cap['lateral_error']:.2f} m, vz {cap['vz']:.2f} m/s, "
              f"propellant left {cap['propellant_left'] / 1e3:.2f} t, time {cap['time']:.1f} s")
    return runs


# ---------------------------------------------------------------- problem geometry (to scale)
def fig_geometry(runs):
    cz = TOWER.catch_point[2]
    gz = TOWER.gate_point[2]
    x0, x1, z0, z1 = -178, 42, -9, 182
    fig, ax = plt.subplots(figsize=(TEXT, 6.0))
    ax.set_aspect("equal")
    # sea offshore of the shoreline, ground inshore
    ax.add_patch(mpatches.Rectangle((x0, z0), -75 - x0, -z0, color="#0b1720", lw=0))
    ax.add_patch(mpatches.Rectangle((-75, z0), x1 + 75, -z0, color="#17140f", lw=0))
    ax.text(-128, -4.6, "Sea", color=FAINT, fontsize=NOTE, ha="center", va="center")
    ax.text(-36, -4.6, "Ground", color=FAINT, fontsize=NOTE, ha="center", va="center")
    # glide-slope cone over the catch point: the region below it is not allowed
    t = np.tan(np.radians(20))
    xs = np.linspace(x0, 0, 200)
    ax.fill_between(xs, 0, cz + t * (-xs), facecolor=BG, edgecolor="#262626", hatch="////", lw=0, zorder=0)
    ax.plot(xs, cz + t * (-xs), color=FAINT, lw=1, ls=(0, (4, 3)), zorder=2)
    ax.text(-150, cz + t * 150 - 11, "Below the 20° glide slope: not allowed", color=DIM, fontsize=NOTE, rotation=20,
            rotation_mode="anchor", ha="left", va="center")
    arc = mpatches.Arc((0, cz), 52, 52, theta1=180, theta2=200, color=FAINT, lw=0.9)
    ax.add_patch(arc)
    ax.text(-30, cz + 5.6, "20°", color=DIM, fontsize=NOTE, ha="right", va="center")
    # offshore boundary until the commit window
    ax.plot([-TOWER.standoff] * 2, [0, z1], color=FAINT, lw=1, ls=(0, (1.5, 3)), zorder=2)
    ax.text(-TOWER.standoff - 3, 172, "Offshore of this line until\nthe commit window opens", color=DIM,
            fontsize=NOTE, ha="right", va="top", linespacing=1.3)
    # tower, arms, launch mount, and the booster's outline at capture
    draw_tower(ax)
    ax.text(15, 149, "Tower", color=DIM, fontsize=NOTE, ha="center", va="bottom")
    ax.add_patch(mpatches.FancyBboxPatch((-4.5, cz - 35.5), 9, 71, boxstyle="round,pad=0,rounding_size=2.5",
                                         facecolor="none", edgecolor="#4d4d4d", lw=1, ls=(0, (3, 2)), zorder=2))
    # trajectory (nominal convex MPC flight) with a direction arrow
    r = runs["mpc"].r
    sel = (r[:, 0] >= x0) & (r[:, 2] <= z1 + 20) & (r[:, 2] >= gz - 0.5)
    ax.plot(r[sel, 0], r[sel, 2], color=COL["mpc"], lw=1.9, zorder=3, solid_capstyle="round")
    k = np.argmin(np.abs(r[:, 2] - 150))
    ax.annotate("", xy=(r[k + 2, 0], r[k + 2, 2]), xytext=(r[k - 2, 0], r[k - 2, 2]),
                arrowprops=dict(arrowstyle="-|>", color=COL["mpc"], lw=0, mutation_scale=13), zorder=4)
    say(ax, (r[k, 0], r[k, 2]), "Nominal convex MPC flight", COL["mpc"], dx=10, dy=6, ha="left", va="bottom")
    ax.plot([0, 0], [cz, gz], color=INK, lw=1.2, ls=(0, (2, 2)), zorder=3)
    # markers and their labels (left of the booster, with leaders)
    ax.plot(0, gz, marker="^", color=INK, ms=6.5, ls="none", zorder=5)
    ax.plot(0, cz, marker="x", color=INK, ms=7.5, mew=1.6, ls="none", zorder=5)
    lead = dict(arrowstyle="-", color=FAINT, lw=0.7, shrinkA=0, shrinkB=4)
    ax.annotate("Terminal gate: 12 m above, passed at 5 m/s", (0, gz), xytext=(-48, 104), textcoords="data",
                ha="right", va="center", color=INK, fontsize=ANN, arrowprops=lead)
    ax.annotate("Catch point: centre of mass at capture", (0, cz), xytext=(-48, 50), textcoords="data",
                ha="right", va="center", color=INK, fontsize=ANN, arrowprops=lead)
    ax.annotate("Arms grip ~30 m above it", (-4, cz + 30), xytext=(-48, 128), textcoords="data",
                ha="right", va="center", color=INK, fontsize=ANN, arrowprops=lead)
    ax.annotate("Booster outline at capture", (-4.5, cz - 26), xytext=(-48, 22), textcoords="data",
                ha="right", va="center", color=DIM, fontsize=NOTE, arrowprops=lead)
    ax.text(x0 + 3, z1 - 3, "From ignition: 1,250 m up and 280 m offshore, falling at 270 m/s",
            color=DIM, fontsize=NOTE, ha="left", va="top")
    ax.set(xlim=(x0, x1), ylim=(z0, z1), xlabel="Distance towards the tower (m)", ylabel="Altitude (m)")
    ax.set_xticks(range(-150, 41, 50))
    ax.set_yticks(range(0, 181, 50))
    ax.grid(False)
    ax.spines["bottom"].set_visible(False)
    save(fig, "geometry.png")


# ---------------------------------------------------------------- nominal flights
def fig_trajectories(runs):
    fig, axs = plt.subplots(2, 2, figsize=(FULL, 6.4))
    ax = axs[0, 0]
    for key, res in runs.items():
        ax.plot(res.r[:, 0], res.r[:, 2], color=COL[key], zorder=3)
    draw_tower(ax)
    r0 = runs["mpc"].r[0]
    ax.plot(r0[0], r0[2], "o", color=INK, ms=5, zorder=4)
    say(ax, (r0[0], r0[2]), "Ignition", INK, dx=8, dy=2)
    m, b = runs["mpc"].r, runs["baseline"].r
    km = np.argmin(np.abs(m[:, 0] + 175))
    kb = np.argmin(np.abs(b[:, 0] + 222))
    say(ax, (m[km, 0], m[km, 2]), "Convex MPC", COL["mpc"], dx=4, dy=12, va="bottom", weight="medium")
    ax.text(-292, 860, "Baseline", color=COL["baseline"], fontsize=ANN, fontweight="medium", va="center")
    ax.axvline(-TOWER.standoff, color=FAINT, ls=(0, (1.5, 3)), lw=1)
    ax.text(-TOWER.standoff - 6, 1150, "Offshore until the\ncommit window", ha="right", va="top", color=DIM,
            fontsize=NOTE, linespacing=1.3)
    ax.set(xlabel="Distance towards the tower (m)", ylabel="Altitude (m)", xlim=(-300, 40), ylim=(0, 1330), title="Side view")
    ax = axs[0, 1]
    for key, res in runs.items():
        sel = res.r[:, 2] < 200
        ax.plot(res.r[sel, 0], res.r[sel, 2], color=COL[key], zorder=3)
    draw_tower(ax)
    ax.plot(*TOWER.gate_point[[0, 2]], marker="^", color=INK, ms=6, ls="none", zorder=4)
    ax.plot(TOWER.catch_point[0], TOWER.catch_point[2], marker="x", color=INK, ms=7, mew=1.4, ls="none", zorder=4)
    say(ax, tuple(TOWER.gate_point[[0, 2]]), "Gate", dx=-9, ha="right", size=NOTE)
    say(ax, (TOWER.catch_point[0], TOWER.catch_point[2]), "Catch point", dx=9, size=NOTE)
    say(ax, (-4, TOWER.catch_point[2] + 30), "Arms", dx=-6, ha="right", size=NOTE)
    ax.text(15, 150, "Tower", ha="center", va="bottom", color=DIM, fontsize=NOTE)
    ax.set(xlabel="Distance towards the tower (m)", ylabel="Altitude (m)", xlim=(-60, 25), ylim=(40, 200), title="Final 200 m")
    ax = axs[1, 0]
    for key, res in runs.items():
        ax.plot(res.t, -res.v[:, 2], color=COL[key])
        ax.plot(res.t, np.linalg.norm(res.v[:, :2], axis=1), color=COL[key], ls=(0, (3, 2)), lw=1.2)
    ax.text(6.2, 140, "Descent rate", color=DIM, fontsize=NOTE, ha="left", va="bottom")
    ax.text(0.35, 13.5, "Lateral speed (dashed)", color=DIM, fontsize=NOTE, ha="left", va="top")
    ax.set(xlabel="Time since ignition (s)", ylabel="Speed (m/s)", yscale="log", ylim=(0.5, 400), xlim=(0, 20), title="Speed")
    plain_log(ax.yaxis, [1, 3, 10, 30, 100, 300])
    ax.set_xticks(range(0, 21, 5))
    ax = axs[1, 1]
    for key, res in runs.items():
        ax.plot(res.t, np.linalg.norm(res.thrust, axis=1) / 1e6, color=COL[key], zorder=3)
    for n, t0, t1 in ((13, 0, NOMINAL.switch_time), (3, NOMINAL.switch_time, 20)):
        lo, hi = SUPER_HEAVY.thrust_bounds(n)
        ax.fill_between([t0, t1], lo / 1e6, hi / 1e6, color="#1c1c1c", lw=0, zorder=1)
    ax.text(0.25, 31.5, "13-engine range", color=DIM, fontsize=NOTE)
    ax.text(NOMINAL.switch_time + 6, 7.4, "3-engine range", color=DIM, fontsize=NOTE)
    ax.set(xlabel="Time since ignition (s)", ylabel="Thrust (MN)", yscale="log", ylim=(1.5, 40), xlim=(0, 20), title="Thrust")
    plain_log(ax.yaxis, [2, 3, 5, 10, 20, 30])
    ax.set_xticks(range(0, 21, 5))
    save(fig, "trajectories.png")


# ---------------------------------------------------------------- trade-off frontier (tuning set)
def fig_tradeoff():
    rows = load("mc_tune_v4.jsonl")
    if not rows:
        return None
    by = {}
    for d in rows:
        by.setdefault(d["controller"], []).append(d)
    # mc_tune_v4.jsonl was flown when the approach cap was on by default, so there a key without
    # "approach_cap" is a capped setting; in a file flown with today's defaults the capped
    # settings say "approach_cap=1" explicitly
    legacy = any("approach_cap=0" in k for k in by)
    is_capped = lambda k: family(k) == "mpc" and ("approach_cap=1" in k or (legacy and "approach_cap" not in k))
    pts = []
    for key, rs in by.items():
        k = sum(d["caught"] for d in rs)
        lo, hi = wilson(k, len(rs))
        prop = [d["propellant_left"] / 1e3 for d in rs if d["caught"]]
        pts.append(dict(key=key, fam=family(key), n=len(rs), rate=k / len(rs), lo=lo, hi=hi,
                        prop=float(np.median(prop)) if prop else np.nan, capped=is_capped(key)))
    fig, ax = plt.subplots(figsize=(TEXT, 4.1))
    base = sorted([p for p in pts if p["fam"] == "baseline"], key=lambda p: p["prop"])
    ax.plot([p["prop"] for p in base], [100 * p["rate"] for p in base], color=COL["baseline"], lw=1, alpha=0.45, zorder=2)
    for p in pts:
        hollow = p["fam"] == "mpc" and p["capped"]
        c = COL[p["fam"]]
        ax.errorbar(p["prop"], 100 * p["rate"], yerr=[[100 * (p["rate"] - p["lo"])], [100 * (p["hi"] - p["rate"])]],
                    fmt="o", color=c, ecolor=c, elinewidth=0.9, capsize=0, ms=6.5, mfc=BG if hollow else c, mew=1.5, zorder=3)
        if p["fam"] == "baseline":
            say(ax, (p["prop"], 100 * p["rate"]), f"{p['key'].split('a2=')[1].rstrip(']')} m/s²", COL["baseline"], dx=8, dy=-3,
                size=NOTE)
        elif not hollow:
            m = round(float(p["key"].split("margin=")[1].split(",")[0].split(";")[0]) * 100)
            dx, dy, ha = {4: (8, -3, "left"), 8: (0, 10, "center"), 12: (-9, -3, "right")}[m]
            say(ax, (p["prop"], 100 * p["rate"]), f"{m}%", COL["mpc"], dx=dx, dy=dy, ha=ha, size=NOTE)
    capped = [p for p in pts if p["fam"] == "mpc" and p["capped"]]
    if capped:
        x, y = np.mean([p["prop"] for p in capped]), np.mean([100 * p["rate"] for p in capped])
        ax.annotate("With approach cap", (x - 0.03, y + 0.8), textcoords="offset points", xytext=(-26, 20), ha="right",
                    fontsize=NOTE, color=COL["mpc"], arrowprops=dict(arrowstyle="-", color=COL["mpc"], lw=0.7, shrinkA=2, shrinkB=4))
    say(ax, (5.27, 70.5), "Convex MPC", COL["mpc"], ha="center", va="top", weight="medium")
    say(ax, (2.04, 62.5), "Baseline", COL["baseline"], ha="center", va="top", weight="medium")
    ax.annotate("Better", xy=(2.38, 100.8), xytext=(1.98, 95.2), color=DIM, fontsize=NOTE, ha="right", va="center",
                arrowprops=dict(arrowstyle="-|>", color=FAINT, lw=0.9, mutation_scale=9, shrinkA=3, shrinkB=0))
    ax.text(0.995, 0.02, "Point labels: thrust margin (MPC) and braking level (baseline)", transform=ax.transAxes,
            ha="right", va="bottom", color=FAINT, fontsize=NOTE)
    ax.set(xlabel="Median propellant left at the catch (t)", ylabel="Catch rate (%)", ylim=(25, 104), xlim=(1.6, 5.9))
    save(fig, "tradeoff.png")
    return pts


# ---------------------------------------------------------------- main Monte Carlo
def fig_outcomes(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, 2, figsize=(FULL, 3.1), gridspec_kw={"width_ratios": [1.1, 1]})
    ax = axs[0]
    y = np.arange(len(keys))[::-1]
    for yi, key in zip(y, keys):
        rs = [d for d in rows if d["controller"] == key]
        oc = Counter(outcome(d) for d in rs)
        left = 0
        for o in ("caught", "safe miss", "pad strike"):
            w = 100 * oc.get(o, 0) / len(rs)
            ax.barh(yi, w, left=left, color=OC_COL[o], height=0.62)
            if w > 5.5:
                ax.text(left + w / 2, yi, f"{oc.get(o, 0)}", ha="center", va="center", color="white", fontsize=NOTE)
            left += w
    ax.set_yticks(y, [SHORT.get(k, label(k)) for k in keys])
    ax.set(xlim=(0, 100), xlabel="Share of 300 flights (%)")
    hbar_axes(ax)
    ax.legend(handles=[mpatches.Patch(color=OC_COL[o], label=OC_NAME[o]) for o in OC_COL], loc="lower left",
              bbox_to_anchor=(0, 1.0), ncol=3, handlelength=1.0, handleheight=0.8, columnspacing=1.2, borderaxespad=0.3)
    ax.set_title("Outcome", pad=30)
    # why the misses happened: a count matrix (rows: reason, columns: controller)
    ax = axs[1]
    reasons = Counter()
    for d in rows:
        if not d["caught"]:
            reasons[(d["controller"], d["reason"].split(";")[0].split(":")[0])] += 1
    pretty = {"aborted (no feasible trajectory)": "Abort: no feasible plan", "aborted (braking out of reach)": "Abort: braking out of reach",
              "aborted (no-go at the gate)": "Abort: no-go at the gate", "out of propellant": "Ran out of propellant",
              "missed": "Missed the tolerance", "hit the tower": "Hit the tower"}
    labels = sorted({r for (_, r) in reasons}, key=lambda r: -sum(v for (k, rr), v in reasons.items() if rr == r))
    M = np.array([[reasons.get((k, r), 0) for k in keys] for r in labels])
    top = M.max()
    for i in range(len(labels)):
        for j in range(len(keys)):
            v = M[i, j]
            if v == 0:
                ax.text(j, i, "–", ha="center", va="center", color="#3a3a3a", fontsize=NOTE)
                continue
            a = 0.16 + 0.84 * np.sqrt(v / top)
            ax.add_patch(mpatches.FancyBboxPatch((j - 0.44, i - 0.36), 0.88, 0.72, boxstyle="round,pad=0,rounding_size=0.08",
                                                 facecolor=(0.93, 0.93, 0.93, a), lw=0))
            ax.text(j, i, str(v), ha="center", va="center", color="#000" if a > 0.5 else INK, fontsize=NOTE)
    ax.set_xlim(-0.5, len(keys) - 0.5)
    ax.set_ylim(len(labels) - 0.5, -0.5)
    ax.set_yticks(range(len(labels)), [pretty.get(r, r) for r in labels])
    cols = {"mpc": "Convex\nMPC", "mpc[approach_cap=1]": "MPC +\ncap", "mpc[eo_reserve=1]": "MPC +\nreserve", "baseline": "Baseline"}
    ax.set_xticks(range(len(keys)), [cols.get(k, k) for k in keys], fontsize=NOTE, linespacing=1.15)
    for tl, k in zip(ax.get_xticklabels(), keys):
        tl.set_color(color(k))
    ax.tick_params(axis="both", length=0)
    ax.grid(False)
    ax.spines["bottom"].set_visible(False)
    ax.set_title("Why flights were not caught", pad=30)
    save(fig, "outcomes.png")


def fig_propellant(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, ax = plt.subplots(figsize=(TEXT, 2.6))
    y = np.arange(len(keys))[::-1]
    for yi, key in zip(y, keys):
        prop = np.array([d["propellant_left"] / 1e3 for d in rows if d["controller"] == key and d["caught"]])
        q5, q25, q50, q75, q95 = np.percentile(prop, [5, 25, 50, 75, 95])
        c = color(key)
        ax.plot([q5, q95], [yi, yi], color=c, lw=1.2, solid_capstyle="butt")
        ax.add_patch(mpatches.Rectangle((q25, yi - 0.22), q75 - q25, 0.44, facecolor=c, alpha=0.32, edgecolor=c, lw=1.1))
        ax.plot([q50, q50], [yi - 0.22, yi + 0.22], color=INK, lw=1.8, solid_capstyle="butt")
        ax.text(q95 + 0.14, yi, f"{q50:.1f} t", va="center", color=INK, fontsize=ANN)
        ax.text(q95 + 0.14 + 0.62, yi, f"median, {len(prop)} catches", va="center", color=FAINT, fontsize=NOTE)
    ax.set_yticks(y, [SHORT.get(k, label(k)) for k in keys])
    ax.set(xlim=(0, 10), ylim=(-0.6, len(keys) - 0.4), xlabel="Propellant left at the catch (t)")
    ax.set_xticks(range(0, 11, 2))
    hbar_axes(ax)
    save(fig, "propellant.png")


def fig_precision(name="mc_random.jsonl"):
    """Cumulative distributions: the share of caught flights within a given distance / lateral speed."""
    rows = load(name)
    if not rows:
        return
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, 2, figsize=(FULL, 3.3))
    for key in keys[::-1]:
        rs = [d for d in rows if d["controller"] == key and d.get("lateral_error") is not None and d["caught"]]
        for ax, vals in ((axs[0], [d["lateral_error"] for d in rs]), (axs[1], [d["vh"] for d in rs])):
            v = np.sort(vals)
            ax.step(np.r_[v[0], v], np.r_[0, np.arange(1, len(v) + 1) / len(v) * 100], where="post", color=color(key),
                    lw=1.8 if "[" not in key else 1.2, label=SHORT.get(key, label(key)))
    for ax in axs:
        ax.axhline(50, color=FAINT, lw=0.8, ls=(0, (1.5, 3)), zorder=0)
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.set_ylim(0, 104)
        ax.grid(axis="x", visible=True)
    ax = axs[0]
    ax.set_xscale("log")
    plain_log(ax.xaxis, [0.01, 0.03, 0.1, 0.3, 1], lambda v, _: {0.01: "1 cm", 0.03: "3 cm", 0.1: "10 cm", 0.3: "30 cm", 1: "1 m"}[v])
    ax.set_xlim(0.004, 1.35)
    ax.axvline(TOWER.tol_lateral, color=DIM, lw=1, ls=(0, (3, 3)))
    say(ax, (TOWER.tol_lateral, 8), "Tolerance", dx=-5, ha="right", size=NOTE)
    for key in ("mpc", "baseline"):
        med = np.median([d["lateral_error"] for d in rows if d["controller"] == key and d["caught"]])
        ax.plot(med, 50, "o", color=color(key), ms=5, zorder=4)
        say(ax, (med, 50), f"median {100 * med:.0f} cm", color(key), dx=-8, dy=9, ha="right", size=NOTE)
    ax.set(xlabel="Distance from the catch point (log scale)", ylabel="Share of caught flights (%)",
           title="Distance from the catch point")
    ax = axs[1]
    ax.set_xlim(0, 0.53)
    ax.axvline(TOWER.tol_lateral_speed, color=DIM, lw=1, ls=(0, (3, 3)))
    say(ax, (TOWER.tol_lateral_speed, 8), "Tolerance", dx=-5, ha="right", size=NOTE)
    ax.set(xlabel="Lateral speed at capture (m/s)", title="Lateral speed at capture")
    fig.legend(handles=[mlines.Line2D([], [], color=color(k), lw=1.8, label=SHORT.get(k, label(k))) for k in keys],
               loc="outside upper right", ncol=4)
    save(fig, "precision.png")


# ---------------------------------------------------------------- wind
def fig_wind():
    rows = load("mc_wind.jsonl")
    if not rows:
        return None
    fig, ax = plt.subplots(figsize=(TEXT, 3.1))
    ax.axvspan(20, 25, color="#111111", lw=0, zorder=0)
    ax.text(22.5, 2, "Both fail\nbetween\n20 and 25", ha="center", va="bottom", color=FAINT, fontsize=NOTE, linespacing=1.25)
    out = {}
    for i, key in enumerate(ctl_keys(rows)):
        speeds = sorted({d["wind_speed"] for d in rows if d["controller"] == key})
        rate, lo, hi, strike = [], [], [], []
        for s in speeds:
            rs = [d for d in rows if d["controller"] == key and d["wind_speed"] == s]
            k = sum(d["caught"] for d in rs)
            a, b = wilson(k, len(rs))
            rate.append(100 * k / len(rs)); lo.append(100 * a); hi.append(100 * b)
            strike.append(sum(outcome(d) == "pad strike" for d in rs))
        xs = np.array(speeds) + (i - 0.5) * 0.45
        ax.errorbar(xs, rate, yerr=[np.array(rate) - lo, np.array(hi) - rate], color=color(key), ecolor=color(key),
                    elinewidth=0.9, capsize=0, marker="o", ms=4.5)
        say(ax, (xs[-1], rate[-1]), SHORT.get(key, label(key)), color(key), dx=10, dy=6 if i == 0 else -4,
            weight="medium")
        out[key] = dict(speeds=speeds, rate=rate, strikes=strike)
    ax.set(xlabel="Mean wind speed (m/s)", ylabel="Catch rate (%)", ylim=(-4, 106), xlim=(-0.8, 37))
    ax.set_xticks(range(0, 31, 5))
    ax.spines["bottom"].set_bounds(-0.8, 30.8)
    save(fig, "wind.png")
    return out


# ---------------------------------------------------------------- engine failure timing
def fig_engineout():
    rows = load("mc_engineout.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, ax = plt.subplots(figsize=(FULL, 0.8 + 0.44 * len(keys)))
    y = np.arange(len(keys))[::-1]
    for yi, key in zip(y, keys):
        rs = [d for d in rows if d["controller"] == key and d["engine_out_time"] is not None]
        for d in rs:
            ax.scatter(d["engine_out_time"], yi, s=50, marker="s", color=OC_COL[outcome(d)], lw=0)
        oc = Counter(outcome(d) for d in rs)
        ax.text(1.012, yi, f"{oc['caught']} caught", transform=ax.get_yaxis_transform(), ha="left", va="center",
                color=INK, fontsize=NOTE)
        ax.text(1.012 + 0.098, yi, f"{oc['pad strike']} pad strikes", transform=ax.get_yaxis_transform(), ha="left",
                va="center", color=OC_COL["pad strike"] if oc["pad strike"] else FAINT, fontsize=NOTE)
    ax.axvline(NOMINAL.switch_time, color=FAINT, ls=(0, (1.5, 3)), lw=1, zorder=0)
    ax.text(NOMINAL.switch_time + 0.12, len(keys) - 0.42, "13 → 3 engines", color=DIM, fontsize=NOTE, va="center")
    ax.set_yticks(y, [SHORT.get(k, label(k)) for k in keys])
    ax.set(xlabel="Time of the engine failure (s after ignition)", ylim=(-0.6, len(keys) - 0.15), xlim=(-0.5, 16.5))
    ax.set_xticks(range(0, 17, 2))
    ax.grid(False)
    fig.legend(handles=[mpatches.Patch(color=c, label=OC_NAME[o]) for o, c in OC_COL.items()], loc="outside upper left",
               ncol=3, handlelength=1.0, handleheight=0.8)
    save(fig, "engineout.png")


# ---------------------------------------------------------------- capture basin
def fig_basin():
    rows = load("mc_basin.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, len(keys), figsize=(FULL, 4.2), sharey=True)
    out = {}
    for ax, key in zip(np.atleast_1d(axs), keys):
        rs = [d for d in rows if d["controller"] == key]
        for d in rs:
            o = outcome(d)
            ax.add_patch(mpatches.FancyBboxPatch((d["dx"] - 16.5, d["dz"] - 21), 33, 42, boxstyle="round,pad=0,rounding_size=2",
                                                 color=OC_COL[o], lw=0))
            if d["caught"]:
                ax.text(d["dx"], d["dz"], f"{d['propellant_left'] / 1e3:.1f}", ha="center", va="center", fontsize=7,
                        color="white")
        ax.add_patch(mpatches.FancyBboxPatch((-18.5, -23), 37, 46, boxstyle="round,pad=0,rounding_size=2.5",
                                             fill=False, edgecolor="white", lw=1.5))
        n_c, n = sum(d["caught"] for d in rs), len(rs)
        n_s = sum(outcome(d) == "pad strike" for d in rs)
        ax.set(xlim=(-220, 220), ylim=(-325, 325), xlabel="Ignition shift towards the tower (m)",
               title=f"{NAME[family(key)]}: {n_c} of {n} caught, {n_s} pad strikes")
        ax.set_xticks(range(-200, 201, 100))
        ax.grid(False)
        ax.spines["bottom"].set_visible(False)
        ax.tick_params(axis="x", length=0)
        out[key] = (n_c, n, n_s)
    np.atleast_1d(axs)[0].set_ylabel("Ignition altitude shift (m)")
    fig.legend(handles=[mpatches.Patch(color=c, label=OC_NAME[o]) for o, c in OC_COL.items()] +
               [mpatches.Patch(facecolor="none", edgecolor="white", lw=1.3, label="Nominal ignition point")],
               loc="outside upper left", ncol=4, handlelength=1.0, handleheight=0.8)
    save(fig, "basin.png")
    return out


# ---------------------------------------------------------------- propellant for engine-out tolerance
def fig_eoprop():
    rows = load("mc_eoprop.jsonl")
    if not rows:
        return None
    fig, axs = plt.subplots(1, 2, figsize=(FULL, 3.1), sharey=True)
    out = {}
    keys = ctl_keys(rows)
    props = sorted({d["landing_propellant"] for d in rows})
    for ax, eo, title in ((axs[0], False, "No engine failure"), (axs[1], True, "One engine fails")):
        for i, key in enumerate(keys):
            xs, ys, lo, hi = [], [], [], []
            for prop in props:
                rs = [d for d in rows if d["controller"] == key and d["landing_propellant"] == prop
                      and (d["d_engine_out_time"] is not None) == eo]
                k = sum(d["caught"] for d in rs)
                a_, b_ = wilson(k, len(rs))
                xs.append(prop / 1e3 + (i - (len(keys) - 1) / 2) * 0.35)
                ys.append(100 * k / len(rs)); lo.append(100 * (k / len(rs) - a_)); hi.append(100 * (b_ - k / len(rs)))
                out[(key, eo, prop)] = (k, len(rs), sum(outcome(d) == "pad strike" for d in rs))
            ax.errorbar(xs, ys, yerr=[lo, hi], color=color(key), ecolor=color(key), elinewidth=0.9, capsize=0,
                        marker="o", ms=5, lw=1.5)
            if eo and "eo_reserve" in key:
                say(ax, ((xs[1] + xs[2]) / 2, ys[1]), "19 of 20 with 5 t more", color(key), dy=-12, ha="center", va="top", size=NOTE)
            if eo and "eo_reserve" not in key:
                say(ax, ((xs[0] + xs[1]) / 2 + 0.3, ys[1]), "4 of 20 at every level", color(key), dy=9, ha="center", va="bottom", size=NOTE)
        ax.set(xlabel="Landing propellant (t)", ylim=(-4, 106), xlim=(43.5, 56.5), title=title)
        ax.set_xticks([p / 1e3 for p in props])
    axs[0].set_ylabel("Catch rate (%)")
    fig.legend(handles=[mlines.Line2D([], [], color=color(k), marker="o", ms=5, lw=1.5,
                                      label="MPC + engine-out reserve" if "eo_reserve" in k else "Convex MPC") for k in keys],
               loc="outside upper right", ncol=2)
    save(fig, "eoprop.png")
    return out


if __name__ == "__main__":
    runs = nominal_runs()
    fig_trajectories(runs)
    fig_tradeoff()
    fig_outcomes()
    fig_propellant()
    fig_precision()
    fig_wind()
    fig_engineout()
    fig_basin()
    fig_eoprop()
    print("figures in", OUT)
