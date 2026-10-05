"""All figures for the report, from the results/ files. Missing inputs are skipped.

    python scripts/make_figures.py

Every figure is drawn at the size the page shows it (100 CSS px per inch; FULL is the width
of the content column, TEXT the width of a text line), so 9 pt type reads the same size in
every figure. Black background and Inter, to sit on the report page.
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
COL = {"mpc": "#56B4E9", "baseline": "#E69F00"}
NAME = {"mpc": "Convex MPC", "baseline": "Baseline"}
OC_COL = {"caught": "#14A374", "safe miss": "#5c5c5c", "pad strike": "#E2652A"}
OC_NAME = {"caught": "Caught", "safe miss": "Safe miss", "pad strike": "Pad strike"}
plt.rcParams.update({
    "figure.dpi": 100, "savefig.dpi": 200, "figure.constrained_layout.use": True,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "font.family": "Inter", "font.size": 9.8, "axes.unicode_minus": True,
    "text.color": INK, "axes.labelcolor": DIM, "axes.labelsize": 9.6, "axes.labelpad": 6,
    "axes.titlesize": 10.3, "axes.titleweight": "medium", "axes.titlelocation": "left", "axes.titlepad": 10,
    "axes.titlecolor": INK, "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "axes.spines.left": False,
    "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "xtick.color": AXIS, "ytick.color": AXIS, "xtick.labelcolor": DIM, "ytick.labelcolor": DIM,
    "xtick.labelsize": 9.2, "ytick.labelsize": 9.2, "xtick.major.size": 3.5, "ytick.major.size": 0,
    "xtick.major.width": 0.8, "xtick.minor.size": 0, "ytick.minor.size": 0, "ytick.major.pad": 6,
    "legend.frameon": False, "legend.fontsize": 9.2, "legend.labelcolor": INK, "legend.handlelength": 1.6,
    "legend.borderaxespad": 0.2, "legend.columnspacing": 1.4,
    "lines.linewidth": 1.7, "lines.markersize": 5, "patch.linewidth": 0, "hatch.color": FAINT,
})


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)


def plain_log(axis, ticks):
    """Log axis labelled 1, 10, 100 rather than 10^0, 10^1, 10^2."""
    axis.set_major_locator(mticker.FixedLocator(ticks))
    axis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))
    axis.set_minor_locator(mticker.NullLocator())


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


def draw_tower(ax):
    ax.add_patch(mpatches.Rectangle((9, 0), 12, 146, color="#2b2b2b", lw=0, zorder=1))
    ax.plot([-4, 9], [TOWER.catch_point[2] + 30] * 2, color="#bdbdbd", lw=1.4, solid_capstyle="butt", zorder=2)
    ax.add_patch(mpatches.Rectangle((-12, 0), 24, 20, color="#1f1f1f", lw=0, zorder=1))   # launch mount


# ---------------------------------------------------------------- nominal flights
def fig_trajectories():
    runs = {}
    for key, ctl in (("mpc", C.ConvexMPC()), ("baseline", C.Baseline())):
        runs[key] = (sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, record_every=2), ctl)
    fig, axs = plt.subplots(2, 2, figsize=(FULL, 6.2))
    ax = axs[0, 0]
    for key, (res, _) in runs.items():
        ax.plot(res.r[:, 0], res.r[:, 2], color=COL[key], label=NAME[key], zorder=3)
    draw_tower(ax)
    ax.axvline(-TOWER.standoff, color=FAINT, ls=(0, (2, 3)), lw=0.9)
    ax.text(-TOWER.standoff - 6, 560, "Offshore until the\ncommit window", ha="right", va="center", color=DIM, fontsize=8.8)
    ax.set(xlabel="Distance towards the tower (m)", ylabel="Altitude (m)", xlim=(-300, 40), ylim=(0, 1320), title="Side view")
    ax.legend(loc="lower left")
    ax = axs[0, 1]
    for key, (res, _) in runs.items():
        sel = res.r[:, 2] < 200
        ax.plot(res.r[sel, 0], res.r[sel, 2], color=COL[key], zorder=3)
    draw_tower(ax)
    ax.plot(*TOWER.gate_point[[0, 2]], marker="^", color=INK, ms=6, ls="none", zorder=4)
    ax.text(TOWER.gate_point[0] - 2.5, TOWER.gate_point[2], "Gate", ha="right", va="center", color=DIM, fontsize=8.8)
    ax.plot(TOWER.catch_point[0], TOWER.catch_point[2], marker="x", color=INK, ms=7, mew=1.4, ls="none", zorder=4)
    ax.text(2.5, TOWER.catch_point[2] - 1, "Catch point", va="center", color=DIM, fontsize=8.8)
    ax.text(-5.5, TOWER.catch_point[2] + 30, "Arms", ha="right", va="center", color=DIM, fontsize=8.8)
    ax.text(15, 150, "Tower", ha="center", va="bottom", color=DIM, fontsize=8.8)
    ax.set(xlabel="Distance towards the tower (m)", ylabel="Altitude (m)", xlim=(-60, 25), ylim=(40, 200), title="Final 200 m")
    ax = axs[1, 0]
    for key, (res, _) in runs.items():
        ax.plot(res.t, -res.v[:, 2], color=COL[key])
        ax.plot(res.t, np.linalg.norm(res.v[:, :2], axis=1), color=COL[key], ls=(0, (3, 2)), lw=1.2)
    ax.set(xlabel="Time since ignition (s)", ylabel="Speed (m/s)", yscale="log", ylim=(0.5, 400), xlim=(0, 20), title="Speed")
    plain_log(ax.yaxis, [1, 3, 10, 30, 100, 300])
    ax.set_xticks(range(0, 21, 5))
    ax.legend(handles=[mlines.Line2D([], [], color=DIM, label="Descent rate"),
                       mlines.Line2D([], [], color=DIM, ls=(0, (3, 2)), lw=1.2, label="Lateral speed")], loc="lower left")
    ax = axs[1, 1]
    for key, (res, _) in runs.items():
        ax.plot(res.t, np.linalg.norm(res.thrust, axis=1) / 1e6, color=COL[key], zorder=3)
    for n, t0, t1 in ((13, 0, NOMINAL.switch_time), (3, NOMINAL.switch_time, 20)):
        lo, hi = SUPER_HEAVY.thrust_bounds(n)
        ax.fill_between([t0, t1], lo / 1e6, hi / 1e6, color="#1c1c1c", lw=0, zorder=1)
    ax.text(0.25, 31.5, "13 engines", color=DIM, fontsize=8.8)
    ax.text(NOMINAL.switch_time + 6, 7.4, "3 engines: feasible range", color=DIM, fontsize=8.8)
    ax.set(xlabel="Time since ignition (s)", ylabel="Thrust (MN)", yscale="log", ylim=(1.5, 40), xlim=(0, 20), title="Thrust")
    plain_log(ax.yaxis, [2, 3, 5, 10, 20, 30])
    ax.set_xticks(range(0, 21, 5))
    for key, (res, _) in runs.items():
        cap = res.capture
        print(f"nominal {key}: {res.reason}, lateral {cap['lateral_error']:.2f} m, vz {cap['vz']:.2f} m/s, "
              f"propellant left {cap['propellant_left'] / 1e3:.2f} t, time {cap['time']:.1f} s")
    save(fig, "trajectories.png")
    return runs


# ---------------------------------------------------------------- trade-off frontier (tuning set)
def fig_tradeoff():
    rows = load("mc_tune_v4.jsonl")
    if not rows:
        return None
    by = {}
    for d in rows:
        by.setdefault(d["controller"], []).append(d)
    pts = []
    for key, rs in by.items():
        k = sum(d["caught"] for d in rs)
        lo, hi = wilson(k, len(rs))
        prop = [d["propellant_left"] / 1e3 for d in rs if d["caught"]]
        pts.append(dict(key=key, fam=family(key), n=len(rs), rate=k / len(rs), lo=lo, hi=hi,
                        prop=float(np.median(prop)) if prop else np.nan, capped="approach_cap=0" not in key))
    fig, ax = plt.subplots(figsize=(TEXT, 4.0))
    base = sorted([p for p in pts if p["fam"] == "baseline"], key=lambda p: p["prop"])
    ax.plot([p["prop"] for p in base], [100 * p["rate"] for p in base], color=COL["baseline"], lw=1, alpha=0.45, zorder=2)
    for p in pts:
        hollow = p["fam"] == "mpc" and p["capped"]
        c = COL[p["fam"]]
        ax.errorbar(p["prop"], 100 * p["rate"], yerr=[[100 * (p["rate"] - p["lo"])], [100 * (p["hi"] - p["rate"])]],
                    fmt="o", color=c, ecolor=c, elinewidth=0.9, capsize=0, ms=6.5, mfc=BG if hollow else c, mew=1.5,
                    alpha=1, zorder=3)
        if p["fam"] == "baseline":
            a2 = p["key"].split("a2=")[1].rstrip("]")
            ax.annotate(f"{a2} m/s²", (p["prop"], 100 * p["rate"]), textcoords="offset points", xytext=(8, -3),
                        fontsize=8.8, color=COL["baseline"])
        elif not hollow:
            m = float(p["key"].split("margin=")[1].split(",")[0].split(";")[0]) * 100
            off = {4: (8, -3), 8: (0, 9), 12: (-8, -3)}[round(m)]
            ax.annotate(f"{m:.0f}%", (p["prop"], 100 * p["rate"]), textcoords="offset points", xytext=off,
                        ha={4: "left", 8: "center", 12: "right"}[round(m)], fontsize=8.8, color=COL["mpc"])
    capped = [p for p in pts if p["fam"] == "mpc" and p["capped"]]
    if capped:
        x, y = np.mean([p["prop"] for p in capped]), np.mean([100 * p["rate"] for p in capped])
        ax.annotate("With approach cap", (x - 0.03, y + 0.8), textcoords="offset points", xytext=(-24, 22), ha="right",
                    fontsize=8.8, color=COL["mpc"], arrowprops=dict(arrowstyle="-", color=COL["mpc"], lw=0.7, shrinkA=2, shrinkB=4))
    ax.set(xlabel="Median propellant left at the catch (t)", ylabel="Catch rate (%)", ylim=(25, 104), xlim=(1.6, 5.9))
    ax.legend(handles=[mlines.Line2D([], [], color=COL["mpc"], marker="o", ls="none", label="Convex MPC, labelled by thrust margin"),
                       mlines.Line2D([], [], color=COL["baseline"], marker="o", ls="none", label="Baseline, labelled by braking level")],
              loc="lower right")
    save(fig, "tradeoff.png")
    return pts


# ---------------------------------------------------------------- main Monte Carlo
SHORT = {"mpc": "Convex MPC", "mpc[approach_cap=1]": "MPC + approach cap", "mpc[eo_reserve=1]": "MPC + engine-out reserve",
         "baseline": "Baseline"}


def fig_outcomes(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, 2, figsize=(FULL, 3.0), gridspec_kw={"width_ratios": [1, 1.1]})
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
                ax.text(left + w / 2, yi, f"{oc.get(o, 0)}", ha="center", va="center", color="white", fontsize=8.8)
            left += w
    ax.set_yticks(y, [SHORT.get(k, label(k)) for k in keys])
    ax.set(xlim=(0, 100), xlabel="Share of 300 flights (%)", title="Outcome")
    ax.grid(axis="y", visible=False)
    ax.grid(axis="x", visible=True)
    ax.tick_params(axis="x", length=0)
    ax.spines["bottom"].set_visible(False)
    ax.legend(handles=[mpatches.Patch(color=OC_COL[o], label=OC_NAME[o]) for o in OC_COL], loc="lower left",
              bbox_to_anchor=(0, 1.0), ncol=3, handlelength=1.0, handleheight=0.8, columnspacing=1.2, borderaxespad=0.3)
    ax.set_title("Outcome", pad=30)
    ax = axs[1]
    reasons = Counter()
    for d in rows:
        if not d["caught"]:
            reasons[(d["controller"], d["reason"].split(";")[0].split(":")[0])] += 1
    pretty = {"aborted (no feasible trajectory)": "Abort: no feasible plan", "aborted (braking out of reach)": "Abort: braking out of reach",
              "aborted (no-go at the gate)": "Abort: no-go at the gate", "out of propellant": "Out of propellant",
              "missed": "Missed the tolerance", "hit the tower": "Hit the tower"}
    labels = sorted({r for (_, r) in reasons}, key=lambda r: -sum(v for (k, rr), v in reasons.items() if rr == r))
    yy = np.arange(len(labels))[::-1]
    h = 0.8 / len(keys)
    for j, key in enumerate(keys):
        ax.barh(yy + (len(keys) / 2 - j - 0.5) * h, [reasons.get((key, r), 0) for r in labels], height=h * 0.92,
                color=color(key), label=SHORT.get(key, label(key)))
    ax.set_yticks(yy, [pretty.get(r, r) for r in labels])
    ax.set(xlabel="Flights not caught", title="Why flights were not caught")
    ax.set_title("Why flights were not caught", pad=30)
    ax.grid(axis="y", visible=False)
    ax.grid(axis="x", visible=True)
    ax.tick_params(axis="x", length=0)
    ax.spines["bottom"].set_visible(False)
    ax.legend(loc="lower right", fontsize=8.8, handlelength=1.0, handleheight=0.8)
    save(fig, "outcomes.png")


def fig_propellant(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, ax = plt.subplots(figsize=(TEXT, 2.5))
    y = np.arange(len(keys))[::-1]
    for yi, key in zip(y, keys):
        prop = np.array([d["propellant_left"] / 1e3 for d in rows if d["controller"] == key and d["caught"]])
        q5, q25, q50, q75, q95 = np.percentile(prop, [5, 25, 50, 75, 95])
        c = color(key)
        ax.plot([q5, q95], [yi, yi], color=c, lw=1.2, solid_capstyle="butt")
        ax.add_patch(mpatches.Rectangle((q25, yi - 0.22), q75 - q25, 0.44, facecolor=c, alpha=0.35, edgecolor=c, lw=1.2))
        ax.plot([q50, q50], [yi - 0.22, yi + 0.22], color=INK, lw=1.8, solid_capstyle="butt")
        ax.text(q95 + 0.12, yi, f"median {q50:.1f} t", va="center", color=DIM, fontsize=8.8)
    ax.set_yticks(y, [SHORT.get(k, label(k)) for k in keys])
    ax.set(xlim=(0, 9), ylim=(-0.6, len(keys) - 0.4), xlabel="Propellant left at the catch (t)")
    ax.grid(axis="y", visible=False)
    ax.grid(axis="x", visible=True)
    ax.tick_params(axis="x", length=0)
    ax.spines["bottom"].set_visible(False)
    save(fig, "propellant.png")


def fig_precision(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return
    fig, axs = plt.subplots(1, 2, figsize=(FULL, 3.2))
    for key in ctl_keys(rows)[::-1]:
        rs = [d for d in rows if d["controller"] == key and d.get("lateral_error") is not None and d["caught"]]
        axs[0].scatter([d["lateral_error"] for d in rs], [d["vh"] for d in rs], s=7, alpha=0.6, color=color(key), lw=0,
                       label=SHORT.get(key, label(key)))
        axs[1].scatter([d["lateral_error"] for d in rs], [-d["vz"] for d in rs], s=7, alpha=0.6, color=color(key), lw=0)
    for ax, tol, ylab, title in ((axs[0], TOWER.tol_lateral_speed, "Lateral speed (m/s)", "Lateral speed at capture"),
                                 (axs[1], TOWER.tol_vertical_speed, "Descent rate (m/s)", "Descent rate at capture")):
        ax.add_patch(mpatches.Rectangle((0, 0), TOWER.tol_lateral, tol, fill=False, ls=(0, (3, 3)), lw=0.9, edgecolor=FAINT))
        ax.text(TOWER.tol_lateral - 0.01, tol * 1.02, "Capture tolerance", ha="right", va="bottom", color=DIM, fontsize=8.8)
        ax.set(xlabel="Distance from the catch point (m)", ylabel=ylab, xlim=(-0.02, 1.05), ylim=(0, tol * 1.12), title=title)
        ax.grid(axis="x", visible=True)
    handles = [mlines.Line2D([], [], color=color(k), marker="o", ls="none", ms=5, label=SHORT.get(k, label(k)))
               for k in ctl_keys(rows)]
    fig.legend(handles=handles, loc="outside upper right", ncol=4, handletextpad=0.2)
    save(fig, "precision.png")


# ---------------------------------------------------------------- wind
def fig_wind():
    rows = load("mc_wind.jsonl")
    if not rows:
        return None
    fig, ax = plt.subplots(figsize=(TEXT, 3.0))
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
                    elinewidth=0.9, capsize=0, marker="o", ms=4.5, label=SHORT.get(key, label(key)))
        out[key] = dict(speeds=speeds, rate=rate, strikes=strike)
    ax.set(xlabel="Mean wind speed (m/s)", ylabel="Catch rate (%)", ylim=(-4, 104), xlim=(-0.8, 30.8))
    ax.set_xticks(range(0, 31, 5))
    ax.legend(loc="lower left")
    save(fig, "wind.png")
    return out


# ---------------------------------------------------------------- engine failure timing
def fig_engineout():
    rows = load("mc_engineout.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, ax = plt.subplots(figsize=(FULL, 0.75 + 0.42 * len(keys)))
    y = np.arange(len(keys))[::-1]
    for yi, key in zip(y, keys):
        for d in rows:
            if d["controller"] == key and d["engine_out_time"] is not None:
                ax.scatter(d["engine_out_time"], yi, s=50, marker="s", color=OC_COL[outcome(d)], lw=0)
    ax.axvline(NOMINAL.switch_time, color=FAINT, ls=(0, (2, 3)), lw=0.9, zorder=0)
    ax.text(NOMINAL.switch_time + 0.12, len(keys) - 0.45, "13 → 3 engines", color=DIM, fontsize=8.8, va="center")
    ax.set_yticks(y, [SHORT.get(k, label(k)) for k in keys])
    ax.set(xlabel="Time of the engine failure (s after ignition)", ylim=(-0.6, len(keys) - 0.2), xlim=(-0.5, 16.5))
    ax.set_xticks(range(0, 17, 2))
    ax.grid(False)
    fig.legend(handles=[mpatches.Patch(color=c, label=OC_NAME[o]) for o, c in OC_COL.items()], loc="outside upper right",
               ncol=3, handlelength=1.0, handleheight=0.8)
    save(fig, "engineout.png")


# ---------------------------------------------------------------- capture basin
def fig_basin():
    rows = load("mc_basin.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, len(keys), figsize=(FULL, 4.1), sharey=True)
    out = {}
    for ax, key in zip(np.atleast_1d(axs), keys):
        rs = [d for d in rows if d["controller"] == key]
        for d in rs:
            o = outcome(d)
            ax.add_patch(mpatches.Rectangle((d["dx"] - 17, d["dz"] - 21.5), 34, 43, color=OC_COL[o], lw=0))
            if d["caught"]:
                ax.text(d["dx"], d["dz"], f"{d['propellant_left'] / 1e3:.1f}", ha="center", va="center", fontsize=7,
                        color="white")
        ax.add_patch(mpatches.Rectangle((-18, -22.5), 36, 45, fill=False, edgecolor="white", lw=1.4))
        n_c, n = sum(d["caught"] for d in rs), len(rs)
        ax.set(xlim=(-220, 220), ylim=(-325, 325), xlabel="Ignition shift towards the tower (m)",
               title=f"{NAME[family(key)]}: {n_c} of {n} caught")
        ax.set_xticks(range(-200, 201, 100))
        ax.grid(False)
        ax.spines["bottom"].set_visible(False)
        ax.tick_params(axis="x", length=0)
        out[key] = (n_c, n, sum(outcome(d) == "pad strike" for d in rs))
    np.atleast_1d(axs)[0].set_ylabel("Ignition altitude shift (m)")
    fig.legend(handles=[mpatches.Patch(color=c, label=OC_NAME[o]) for o, c in OC_COL.items()], loc="outside upper right",
               ncol=3, handlelength=1.0, handleheight=0.8)
    save(fig, "basin.png")
    return out


# ---------------------------------------------------------------- propellant for engine-out tolerance
def fig_eoprop():
    rows = load("mc_eoprop.jsonl")
    if not rows:
        return None
    fig, axs = plt.subplots(1, 2, figsize=(FULL, 3.0), sharey=True)
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
            name = "MPC + engine-out reserve" if "eo_reserve" in key else "Convex MPC"
            ax.errorbar(xs, ys, yerr=[lo, hi], color=color(key), ecolor=color(key), elinewidth=0.9, capsize=0,
                        marker="o", ms=5, lw=1.5, label=name)
        ax.set(xlabel="Landing propellant (t)", ylim=(-4, 104), xlim=(43.5, 56.5), title=title)
        ax.set_xticks([p / 1e3 for p in props])
    axs[0].set_ylabel("Catch rate (%)")
    axs[1].legend(loc="center right", bbox_to_anchor=(1.0, 0.62))
    save(fig, "eoprop.png")
    return out


if __name__ == "__main__":
    fig_trajectories()
    fig_tradeoff()
    fig_outcomes()
    fig_propellant()
    fig_precision()
    fig_wind()
    fig_engineout()
    fig_basin()
    fig_eoprop()
    print("figures in", OUT)
