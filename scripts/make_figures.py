"""All figures for the report, from the results/ files. Missing inputs are skipped.

    python scripts/make_figures.py
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
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.ticker as mticker  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from boostercatch import ROOT, controllers as C, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
from summarize import outcome, wilson  # noqa: E402

RES = ROOT / "results"
OUT = ROOT / "docs" / "images"
OUT.mkdir(parents=True, exist_ok=True)
COL = {"mpc": "#56B4E9", "baseline": "#E69F00"}
NAME = {"mpc": "Convex MPC", "baseline": "Baseline"}
OC_COL = {"caught": "#009E73", "safe miss": "#6f6f6f", "pad strike": "#D55E00"}
INK, DIM, AXIS = "#ececec", "#a3a3a3", "#5c5c5c"
# report style, matching the page: black background, white sans text, no titles inside the
# plots (the captions carry them), panel letters on multi-panel figures, thin axes
plt.rcParams.update({"figure.dpi": 100, "savefig.dpi": 200, "savefig.bbox": "tight", "savefig.pad_inches": 0.06,
                     "figure.facecolor": "#000000", "axes.facecolor": "#000000", "savefig.facecolor": "#000000",
                     "text.color": INK, "axes.labelcolor": INK, "axes.edgecolor": AXIS, "legend.labelcolor": INK,
                     "xtick.color": AXIS, "ytick.color": AXIS, "xtick.labelcolor": DIM, "ytick.labelcolor": DIM,
                     "patch.edgecolor": "#d0d0d0", "grid.color": "#2a2a2a",
                     "font.family": "sans-serif", "font.sans-serif": ["Inter", "Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"],
                     "mathtext.fontset": "stixsans",
                     "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9, "legend.fontsize": 8,
                     "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.linewidth": 0.6,
                     "xtick.direction": "in", "ytick.direction": "in", "xtick.major.width": 0.6, "ytick.major.width": 0.6,
                     "xtick.minor.width": 0.5, "ytick.minor.width": 0.5,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.grid": False,
                     "legend.frameon": False, "lines.linewidth": 1.3, "patch.linewidth": 0.6})


def plain_log(ax, ticks):
    """Log axis labelled 1, 10, 100 rather than 10^0, 10^1, 10^2."""
    ax.yaxis.set_major_locator(mticker.FixedLocator(ticks))
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))
    ax.yaxis.set_minor_formatter(mticker.NullFormatter())


def panel(ax, letter, text=""):
    """'(a) side view' at the top left of a panel."""
    ax.set_title(f"({letter}) {text}".strip(), loc="left", fontsize=9, pad=4)


def load(name):
    f = RES / name
    if not f.exists():
        print("skip (missing)", name)
        return None
    return [json.loads(line) for line in open(f)]


def family(key):
    return key.split("[")[0]


def label(key):
    """'mpc[approach_cap=0]' -> 'Convex MPC (no approach cap)'."""
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
        return "#CC79A7"
    c = COL[family(key)]
    return c if "[" not in key else {"mpc": "#A9D6F5", "baseline": "#F0C36B"}[family(key)]


def draw_tower(ax, view="side"):
    if view == "side":
        ax.add_patch(mpatches.Rectangle((9, 0), 12, 146, color="#3d3d3d", lw=0))
        ax.plot([-4, 9], [TOWER.catch_point[2] + 30] * 2, color="#d0d0d0", lw=1.6)   # arms (schematic)
        ax.add_patch(mpatches.Rectangle((-12, 0), 24, 20, color="#262626", lw=0))  # launch mount
    else:
        ax.add_patch(mpatches.Rectangle((9, -10), 12, 20, color="#3d3d3d", lw=0))
        ax.add_patch(mpatches.Circle((0, 0), sim.PAD_RADIUS, fill=False, ls="--", color="#D55E00", alpha=0.7))


# ---------------------------------------------------------------- 1. nominal flights
def fig_trajectories():
    runs = {}
    for key, ctl in (("mpc", C.ConvexMPC()), ("baseline", C.Baseline())):
        runs[key] = (sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, record_every=2), ctl)
    fig, axs = plt.subplots(2, 2, figsize=(7.4, 5.6))
    ax = axs[0, 0]
    for key, (res, _) in runs.items():
        ax.plot(res.r[:, 0], res.r[:, 2], color=COL[key], label=NAME[key])
    draw_tower(ax)
    ax.axvline(-TOWER.standoff, color=DIM, ls=":", lw=0.8)
    ax.text(-TOWER.standoff - 3, 560, "offshore until the\ncommit window", ha="right", fontsize=7.5)
    ax.set(xlabel="$x$, towards the tower (m)", ylabel="altitude (m)", xlim=(-300, 40))
    panel(ax, "a", "side view")
    ax.legend(loc="upper right")
    ax = axs[0, 1]
    for key, (res, _) in runs.items():
        sel = res.r[:, 2] < 200
        ax.plot(res.r[sel, 0], res.r[sel, 2], color=COL[key])
    draw_tower(ax)
    ax.plot(*TOWER.gate_point[[0, 2]], "^", color=INK, ms=6)
    ax.text(TOWER.gate_point[0] - 2, TOWER.gate_point[2] + 4, "gate", ha="right", fontsize=7.5)
    ax.plot(TOWER.catch_point[0], TOWER.catch_point[2], "x", color=INK, ms=7)
    ax.text(-5, TOWER.catch_point[2] + 32, "arms (catch pins ~30 m\nabove the centre of mass)", ha="right", fontsize=7)
    ax.text(1.5, TOWER.catch_point[2] - 4, "catch point", fontsize=7)
    ax.set(xlabel="$x$ (m)", ylabel="altitude (m)", xlim=(-60, 25), ylim=(40, 200))
    panel(ax, "b", "last 200 m")
    ax = axs[1, 0]
    for key, (res, _) in runs.items():
        ax.plot(res.t, -res.v[:, 2], color=COL[key], label=f"{NAME[key]} descent rate")
        ax.plot(res.t, np.linalg.norm(res.v[:, :2], axis=1), color=COL[key], ls="--", lw=0.9)
    ax.set(xlabel="time since ignition (s)", ylabel="speed (m/s)", yscale="log", ylim=(0.5, 400))
    plain_log(ax, [1, 3, 10, 30, 100, 300])
    panel(ax, "c", "descent rate (solid), lateral speed (dashed)")
    ax = axs[1, 1]
    for key, (res, _) in runs.items():
        ax.plot(res.t, np.linalg.norm(res.thrust, axis=1) / 1e6, color=COL[key])
    for n, t0, t1 in ((13, 0, NOMINAL.switch_time), (3, NOMINAL.switch_time, 22)):
        lo, hi = SUPER_HEAVY.thrust_bounds(n)
        ax.fill_between([t0, t1], lo / 1e6, hi / 1e6, color="#262626", lw=0)
    ax.text(0.2, 31, "13 engines", fontsize=7.5)
    ax.text(NOMINAL.switch_time + 5.0, 7.6, "3 engines", fontsize=7.5)
    ax.set(xlabel="time since ignition (s)", ylabel="thrust (MN)", yscale="log", ylim=(1.5, 40))
    plain_log(ax, [2, 3, 5, 10, 20, 30])
    panel(ax, "d", "thrust and feasible range (grey)")
    for key, (res, _) in runs.items():
        cap = res.capture
        print(f"nominal {key}: {res.reason}, lateral {cap['lateral_error']:.2f} m, vz {cap['vz']:.2f} m/s, "
              f"propellant left {cap['propellant_left'] / 1e3:.2f} t, time {cap['time']:.1f} s")
    fig.tight_layout()
    fig.savefig(OUT / "trajectories.png")
    plt.close(fig)
    return runs


# ---------------------------------------------------------------- 2. trade-off frontier
def fig_tradeoff():
    rows = load("mc_tune_v4.jsonl")
    if not rows:
        return None
    by = {}
    for d in rows:
        by.setdefault(d["controller"], []).append(d)
    pts = []
    for key, rs in by.items():
        n = len(rs)
        k = sum(d["caught"] for d in rs)
        lo, hi = wilson(k, n)
        prop = [d["propellant_left"] / 1e3 for d in rs if d["caught"]]
        strikes = sum(outcome(d) == "pad strike" for d in rs)
        pts.append(dict(key=key, fam=family(key), n=n, rate=k / n, lo=lo, hi=hi, strikes=strikes,
                        prop=float(np.median(prop)) if prop else np.nan))
    fig, ax = plt.subplots(figsize=(6.0, 3.7))
    capped = [p for p in pts if p["fam"] == "mpc" and "approach_cap=0" not in p["key"]]
    for p in pts:
        hollow = p in capped
        ax.errorbar(p["prop"], 100 * p["rate"], yerr=[[100 * (p["rate"] - p["lo"])], [100 * (p["hi"] - p["rate"])]],
                    fmt="o", color=COL[p["fam"]], ms=5.5, capsize=2, lw=0.7,
                    mfc="#000000" if hollow else COL[p["fam"]], mew=1.5)
        if hollow:
            continue
        if p["fam"] == "mpc":
            text = f"MPC, margin {float(p['key'].split('margin=')[1].split(',')[0]) * 100:.0f}%"
        else:
            text = f"$a_2$ = {p['key'].split('a2=')[1].rstrip(']')}"
        if p["strikes"]:
            text += f" ({p['strikes']} pad strike)"
        off, ha = (((0, 8), "center") if "margin=0.08" in p["key"] else ((-7, -3), "right") if "margin=0.12" in p["key"]
                   else ((6, -12), "left") if p["fam"] == "mpc" else ((6, 4), "left"))
        ax.annotate(text, (p["prop"], 100 * p["rate"]), textcoords="offset points", xytext=off, ha=ha,
                    fontsize=7, color=COL[p["fam"]])
    if capped:
        x = np.mean([p["prop"] for p in capped])
        y = max(100 * p["rate"] for p in capped)
        ax.annotate("MPC with approach cap\n(margins 4, 8, 12%; hollow)", (x, y), textcoords="offset points",
                    xytext=(-150, 12), fontsize=7, color=COL["mpc"], arrowprops=dict(arrowstyle="-", color=COL["mpc"], lw=0.5))
    ax.set(xlabel="median propellant left at the catch (t)", ylabel="catch rate (%)")
    ax.legend(handles=[mpatches.Patch(color=COL["mpc"], label="Convex MPC"),
                       mpatches.Patch(color=COL["baseline"], label=r"Baseline ($a_2$: braking level, m/s$^2$)")], loc="lower left")
    fig.tight_layout()
    fig.savefig(OUT / "tradeoff.png")
    plt.close(fig)
    return pts


# ---------------------------------------------------------------- 3. main Monte Carlo
def fig_outcomes(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return None
    fams = ctl_keys(rows)
    fig, axs = plt.subplots(1, 3, figsize=(10.5, 3.5), gridspec_kw={"width_ratios": [1.1, 1.5, 1.2]})
    ax = axs[0]
    for i, key in enumerate(fams):
        rs = [d for d in rows if d["controller"] == key]
        oc = Counter(outcome(d) for d in rs)
        bottom = 0
        for o in ("caught", "safe miss", "pad strike"):
            frac = 100 * oc.get(o, 0) / len(rs)
            ax.bar(i, frac, bottom=bottom, color=OC_COL[o], width=0.6, label=o if i == 0 else None)
            if frac > 3:
                ax.text(i, bottom + frac / 2, f"{oc.get(o, 0)}", ha="center", va="center",
                        color="white", fontsize=7.5)
            bottom += frac
    short = {"mpc": "MPC", "mpc[approach_cap=1]": "MPC\n+ cap", "mpc[eo_reserve=1]": "MPC\n+ reserve", "baseline": "Baseline"}
    ax.set_xticks(range(len(fams)), [short.get(k, label(k)) for k in fams], fontsize=7.5)
    ax.set(ylabel="flights (%)", ylim=(0, 113), yticks=range(0, 101, 20))
    panel(ax, "a", "outcomes")
    ax.legend(loc="upper center", ncol=3, fontsize=7, handlelength=1.0, columnspacing=0.8, borderaxespad=0.1)
    ax = axs[1]
    reasons = Counter()
    for d in rows:
        if not d["caught"]:
            reasons[(d["controller"], d["reason"].split(";")[0].split(":")[0])] += 1
    labels = sorted({r for (_, r) in reasons}, key=lambda r: -sum(v for (k, rr), v in reasons.items() if rr == r))
    y = np.arange(len(labels))
    for j, key in enumerate(fams):
        hgt = 0.76 / len(fams)
        ax.barh(y + (j - (len(fams) - 1) / 2) * hgt, [reasons.get((key, r), 0) for r in labels], height=hgt,
                color=color(key), label=label(key))
    ax.set_yticks(y, labels, fontsize=7.5)
    ax.invert_yaxis()
    ax.set(xlabel="flights")
    panel(ax, "b", "reason for each miss")
    ax.legend(fontsize=7.5)
    ax = axs[2]
    for key in fams:
        prop = [d["propellant_left"] / 1e3 for d in rows if d["controller"] == key and d["caught"]]
        ax.hist(prop, bins=np.arange(0, 9, 0.4), color=color(key), histtype="step", lw=1.2, label=label(key))
    ax.set(xlabel="propellant left at the catch (t)", ylabel="flights")
    panel(ax, "c", "propellant margin of the catches")
    ax.set_ylim(0, ax.get_ylim()[1] * 1.35)
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(OUT / "outcomes.png")
    plt.close(fig)


def fig_precision(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return
    fig, axs = plt.subplots(1, 2, figsize=(7.4, 3.0))
    for key in ctl_keys(rows):
        rs = [d for d in rows if d["controller"] == key and d.get("lateral_error") is not None and d["caught"]]
        axs[0].scatter([d["lateral_error"] for d in rs], [d["vh"] for d in rs], s=5, alpha=0.5, color=color(key),
                       label=label(key), lw=0)
        axs[1].scatter([d["lateral_error"] for d in rs], [-d["vz"] for d in rs], s=5, alpha=0.5, color=color(key), lw=0)
    axs[0].add_patch(mpatches.Rectangle((0, 0), TOWER.tol_lateral, TOWER.tol_lateral_speed, fill=False, ls="--", lw=0.7))
    axs[1].add_patch(mpatches.Rectangle((0, 0), TOWER.tol_lateral, TOWER.tol_vertical_speed, fill=False, ls="--", lw=0.7))
    axs[0].set(xlabel="distance from the catch point (m)", ylabel="lateral speed (m/s)")
    axs[1].set(xlabel="distance from the catch point (m)", ylabel="descent rate (m/s)")
    panel(axs[0], "a", "lateral speed at capture")
    panel(axs[1], "b", "descent rate at capture")
    axs[0].legend(fontsize=7, markerscale=2)
    fig.tight_layout()
    fig.savefig(OUT / "precision.png")
    plt.close(fig)


# ---------------------------------------------------------------- 4. wind
def fig_wind():
    rows = load("mc_wind.jsonl")
    if not rows:
        return None
    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    out = {}
    for key in ctl_keys(rows):
        speeds = sorted({d["wind_speed"] for d in rows if d["controller"] == key})
        rate, lo, hi, strike = [], [], [], []
        for s in speeds:
            rs = [d for d in rows if d["controller"] == key and d["wind_speed"] == s]
            k = sum(d["caught"] for d in rs)
            a, b = wilson(k, len(rs))
            rate.append(100 * k / len(rs)); lo.append(100 * a); hi.append(100 * b)
            strike.append(sum(outcome(d) == "pad strike" for d in rs))
        ax.plot(speeds, rate, "o-", color=color(key), label=label(key), ms=4)
        ax.fill_between(speeds, lo, hi, color=color(key), alpha=0.12)
        out[key] = dict(speeds=speeds, rate=rate, strikes=strike)
    ax.set(xlabel="mean wind speed (m/s)", ylabel="catch rate (%)", ylim=(-3, 103))
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "wind.png")
    plt.close(fig)
    return out


# ---------------------------------------------------------------- 5. engine failure timing
def fig_engineout():
    rows = load("mc_engineout.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)[::-1]   # first controller drawn on top
    fig, ax = plt.subplots(figsize=(7.4, 0.9 + 0.45 * len(keys)))
    for i, key in enumerate(keys):
        for d in rows:
            if d["controller"] != key or d["engine_out_time"] is None:
                continue
            ax.scatter(d["engine_out_time"], i, s=42, marker="s", color=OC_COL[outcome(d)], edgecolor="#000000", lw=0.5)
    ax.axvline(NOMINAL.switch_time, color=DIM, ls=":", lw=0.8)
    ax.text(NOMINAL.switch_time + 0.1, len(keys) - 0.45, r"13 $\rightarrow$ 3 engines", fontsize=7.5)
    ax.set_yticks(range(len(keys)), [label(k) for k in keys])
    ax.set(xlabel="time of the engine failure (s after ignition)", ylim=(-0.6, len(keys) - 0.3))
    ax.grid(False)
    ax.legend(handles=[mpatches.Patch(color=c, label=o) for o, c in OC_COL.items()], loc="upper center",
              bbox_to_anchor=(0.5, -0.42), ncol=3, fontsize=7.5, frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "engineout.png")
    plt.close(fig)


# ---------------------------------------------------------------- 6. capture basin
def fig_basin():
    rows = load("mc_basin.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, len(keys), figsize=(3.7 * len(keys), 3.4), sharey=True)
    out = {}
    for ax, key in zip(np.atleast_1d(axs), keys):
        rs = [d for d in rows if d["controller"] == key]
        for d in rs:
            o = outcome(d)
            ax.add_patch(mpatches.Rectangle((d["dx"] - 18, d["dz"] - 22.5), 36, 45, color=OC_COL[o], alpha=0.85, lw=0))
            if d["caught"]:
                ax.text(d["dx"], d["dz"], f"{d['propellant_left'] / 1e3:.1f}", ha="center", va="center", fontsize=5,
                        color="white")
        ax.add_patch(mpatches.Rectangle((-18, -22.5), 36, 45, fill=False, ec="white", lw=1.2))   # nominal ignition point
        ax.set(xlim=(-220, 220), ylim=(-325, 325), xlabel="ignition shift towards the tower (m)")
        panel(ax, "ab"[list(keys).index(key)], f"{label(key)}: {sum(d['caught'] for d in rs)}/{len(rs)} caught")
        ax.grid(False)
        out[key] = (sum(d["caught"] for d in rs), len(rs), sum(outcome(d) == "pad strike" for d in rs))
    np.atleast_1d(axs)[0].set_ylabel("ignition altitude shift (m)")
    fig.legend(handles=[mpatches.Patch(color=c, label=o) for o, c in OC_COL.items()], loc="lower center", ncol=3,
               fontsize=7.5, frameon=False)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(OUT / "basin.png")
    plt.close(fig)
    return out


# ---------------------------------------------------------------- 7. propellant for engine-out tolerance
def fig_eoprop():
    rows = load("mc_eoprop.jsonl")
    if not rows:
        return None
    fig, ax = plt.subplots(figsize=(5.8, 3.3))
    out = {}
    styles = {True: "-", False: "--"}
    for key in ctl_keys(rows):
        for eo in (True, False):
            xs, ys, lo, hi = [], [], [], []
            for prop in sorted({d["landing_propellant"] for d in rows}):
                rs = [d for d in rows if d["controller"] == key and d["landing_propellant"] == prop
                      and (d["d_engine_out_time"] is not None) == eo]
                k = sum(d["caught"] for d in rs)
                a, b = wilson(k, len(rs))
                xs.append(prop / 1e3); ys.append(100 * k / len(rs)); lo.append(100 * a); hi.append(100 * b)
                out[(key, eo, prop)] = (k, len(rs), sum(outcome(d) == "pad strike" for d in rs))
            ax.plot(xs, ys, "o" + styles[eo], color=color(key), ms=4,
                    label=f"{label(key)}, {'one engine fails' if eo else 'no engine failure'}")
            ax.fill_between(xs, lo, hi, color=color(key), alpha=0.08)
    ax.set(xlabel="landing propellant (t)", ylabel="catch rate (%)", ylim=(-3, 103))
    ax.legend(fontsize=7, loc="center right", bbox_to_anchor=(1.0, 0.55))
    fig.tight_layout()
    fig.savefig(OUT / "eoprop.png")
    plt.close(fig)
    return out


if __name__ == "__main__":
    fig_trajectories()
    fig_tradeoff()
    fig_outcomes()
    fig_precision()
    fig_wind()
    fig_engineout()
    fig_basin()
    fig_eoprop()
    print("figures in", OUT)
