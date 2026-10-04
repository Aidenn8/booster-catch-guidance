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
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from boostercatch import ROOT, controllers as C, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
from summarize import outcome, wilson  # noqa: E402

RES = ROOT / "results"
OUT = ROOT / "docs" / "images"
OUT.mkdir(parents=True, exist_ok=True)
COL = {"mpc": "#2563eb", "baseline": "#ea7a12"}
NAME = {"mpc": "Convex MPC", "baseline": "Baseline"}
OC_COL = {"caught": "#16a34a", "safe miss": "#9ca3af", "pad strike": "#dc2626"}
plt.rcParams.update({"figure.dpi": 130, "font.size": 9.5, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.alpha": 0.25})


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
    rest = rest.replace("approach_cap=0", "no approach cap").replace("margin=", "margin ").replace(",", ", ")
    return f"{name} ({rest})"


def ctl_keys(rows):
    """Controllers in a results file, MPC variants first, plain names before variants."""
    return sorted({d["controller"] for d in rows}, key=lambda k: (family(k) != "mpc", "[" in k, k))


def color(key):
    c = COL[family(key)]
    return c if "[" not in key else {"mpc": "#93b4f5", "baseline": "#f5b77a"}[family(key)]


def draw_tower(ax, view="side"):
    if view == "side":
        ax.add_patch(mpatches.Rectangle((9, 0), 12, 146, color="#6b7280", alpha=0.5, lw=0))
        ax.plot([-4, 9], [TOWER.catch_point[2] + 30] * 2, color="#374151", lw=2)   # arms (schematic)
        ax.add_patch(mpatches.Rectangle((-12, 0), 24, 20, color="#9ca3af", alpha=0.5, lw=0))  # launch mount
    else:
        ax.add_patch(mpatches.Rectangle((9, -10), 12, 20, color="#6b7280", alpha=0.5, lw=0))
        ax.add_patch(mpatches.Circle((0, 0), sim.PAD_RADIUS, fill=False, ls="--", color="#dc2626", alpha=0.6))


# ---------------------------------------------------------------- 1. nominal flights
def fig_trajectories():
    runs = {}
    for key, ctl in (("mpc", C.ConvexMPC()), ("baseline", C.Baseline())):
        runs[key] = (sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, record_every=2), ctl)
    fig, axs = plt.subplots(2, 2, figsize=(10, 7.2))
    ax = axs[0, 0]
    for key, (res, _) in runs.items():
        ax.plot(res.r[:, 0], res.r[:, 2], color=COL[key], label=NAME[key])
    draw_tower(ax)
    ax.axvline(-TOWER.standoff, color="k", ls=":", lw=0.8)
    ax.text(-TOWER.standoff - 3, 560, "stay offshore of this\nline until the\ncommit window", ha="right", fontsize=8)
    ax.set(xlabel="x, towards the tower (m)", ylabel="altitude (m)", title="Side view", xlim=(-300, 40))
    ax.legend(loc="upper right", fontsize=8)
    ax = axs[0, 1]
    for key, (res, _) in runs.items():
        sel = res.r[:, 2] < 200
        ax.plot(res.r[sel, 0], res.r[sel, 2], color=COL[key])
    draw_tower(ax)
    ax.plot(*TOWER.gate_point[[0, 2]], "k^", ms=6)
    ax.text(TOWER.gate_point[0] - 2, TOWER.gate_point[2] + 4, "gate", ha="right", fontsize=8)
    ax.plot(TOWER.catch_point[0], TOWER.catch_point[2], "kx", ms=7)
    ax.set(xlabel="x (m)", ylabel="altitude (m)", title="Last 200 m (close-up)", xlim=(-60, 25), ylim=(40, 200))
    ax = axs[1, 0]
    for key, (res, _) in runs.items():
        ax.plot(res.t, -res.v[:, 2], color=COL[key], label=f"{NAME[key]} descent rate")
        ax.plot(res.t, np.linalg.norm(res.v[:, :2], axis=1), color=COL[key], ls="--", lw=1)
    ax.set(xlabel="time since ignition (s)", ylabel="speed (m/s)", title="Descent rate (solid), sideways speed (dashed)",
           yscale="log", ylim=(0.5, 400))
    ax = axs[1, 1]
    for key, (res, _) in runs.items():
        ax.plot(res.t, np.linalg.norm(res.thrust, axis=1) / 1e6, color=COL[key])
    for n, t0, t1 in ((13, 0, NOMINAL.switch_time), (3, NOMINAL.switch_time, 22)):
        lo, hi = SUPER_HEAVY.thrust_bounds(n)
        ax.fill_between([t0, t1], lo / 1e6, hi / 1e6, color="#d1d5db", alpha=0.5, lw=0)
    ax.text(0.2, 31, "13 engines", fontsize=8)
    ax.text(NOMINAL.switch_time + 0.3, 7.6, "3 engines", fontsize=8)
    ax.set(xlabel="time since ignition (s)", ylabel="thrust (MN)", title="Thrust within the engines' range (grey)",
           yscale="log", ylim=(1.5, 40))
    for key, (res, _) in runs.items():
        cap = res.capture
        print(f"nominal {key}: {res.reason}, lateral {cap['lateral_error']:.2f} m, vz {cap['vz']:.2f} m/s, "
              f"propellant left {cap['propellant_left'] / 1e3:.2f} t, time {cap['time']:.1f} s")
    fig.suptitle("Nominal landing burn: both controllers catch the booster", fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "trajectories.png")
    plt.close(fig)
    return runs


# ---------------------------------------------------------------- 2. trade-off frontier
def fig_tradeoff():
    rows = load("mc_tune_v3.jsonl")
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
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    for p in pts:
        ax.errorbar(p["prop"], 100 * p["rate"], yerr=[[100 * (p["rate"] - p["lo"])], [100 * (p["hi"] - p["rate"])]],
                    fmt="o", color=COL[p["fam"]], ms=6, capsize=2, lw=0.8)
        label = p["key"].replace("baseline", "B").replace("mpc", "MPC").replace("approach_cap=0", "no cap")
        label = label.replace("margin=", "m=").replace("a2=", "a2=")
        ax.annotate(label + (f"  ({p['strikes']} strike)" if p["strikes"] else ""), (p["prop"], 100 * p["rate"]),
                    textcoords="offset points", xytext=(5, 4), fontsize=7.5, color=COL[p["fam"]])
    ax.set(xlabel="median propellant left at the catch (t)", ylabel="catch rate (%)",
           title=f"Fuel vs robustness on the tuning set ({pts[0]['n']} dispersed flights each, 95% intervals)")
    ax.legend(handles=[mpatches.Patch(color=COL[k], label=NAME[k]) for k in COL], loc="lower left")
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
    fig, axs = plt.subplots(1, 3, figsize=(12, 4.0), gridspec_kw={"width_ratios": [1, 1.4, 1.2]})
    ax = axs[0]
    for i, key in enumerate(fams):
        rs = [d for d in rows if d["controller"] == key]
        oc = Counter(outcome(d) for d in rs)
        bottom = 0
        for o in ("caught", "safe miss", "pad strike"):
            frac = 100 * oc.get(o, 0) / len(rs)
            ax.bar(i, frac, bottom=bottom, color=OC_COL[o], width=0.6, label=o if i == 0 else None)
            if frac > 3:
                ax.text(i, bottom + frac / 2, f"{oc.get(o, 0)}", ha="center", va="center", color="white", fontsize=8)
            bottom += frac
    ax.set_xticks(range(len(fams)), [label(k).replace(" (", "\n(") for k in fams], fontsize=8)
    ax.set(ylabel="% of flights", title=f"Outcomes ({len(rows) // len(fams)} dispersed flights each)", ylim=(0, 100))
    ax.legend(loc="lower right", fontsize=8)
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
    ax.set_yticks(y, labels, fontsize=8)
    ax.invert_yaxis()
    ax.set(xlabel="flights", title="Why the misses happened")
    ax.legend(fontsize=8)
    ax = axs[2]
    for key in fams:
        prop = [d["propellant_left"] / 1e3 for d in rows if d["controller"] == key and d["caught"]]
        ax.hist(prop, bins=np.arange(0, 9, 0.4), color=color(key), alpha=0.55, label=label(key))
    ax.set(xlabel="propellant left at the catch (t)", ylabel="flights", title="Fuel margin of the catches")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "outcomes.png")
    plt.close(fig)


def fig_precision(name="mc_random.jsonl"):
    rows = load(name)
    if not rows:
        return
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.8))
    for key in ctl_keys(rows):
        rs = [d for d in rows if d["controller"] == key and d.get("lateral_error") is not None and d["caught"]]
        axs[0].scatter([d["lateral_error"] for d in rs], [d["vh"] for d in rs], s=7, alpha=0.5, color=color(key),
                       label=label(key))
        axs[1].scatter([d["lateral_error"] for d in rs], [-d["vz"] for d in rs], s=7, alpha=0.5, color=color(key))
    axs[0].add_patch(mpatches.Rectangle((0, 0), TOWER.tol_lateral, TOWER.tol_lateral_speed, fill=False, ls="--"))
    axs[1].add_patch(mpatches.Rectangle((0, 0), TOWER.tol_lateral, TOWER.tol_vertical_speed, fill=False, ls="--"))
    axs[0].set(xlabel="distance from the catch point (m)", ylabel="sideways speed (m/s)",
               title="State at the arms (caught flights); box = tolerance")
    axs[1].set(xlabel="distance from the catch point (m)", ylabel="descent rate (m/s)", title="")
    axs[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "precision.png")
    plt.close(fig)


# ---------------------------------------------------------------- 4. wind
def fig_wind():
    rows = load("mc_wind.jsonl")
    if not rows:
        return None
    fig, ax = plt.subplots(figsize=(6.5, 3.8))
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
        ax.plot(speeds, rate, "o-", color=color(key), label=label(key))
        ax.fill_between(speeds, lo, hi, color=color(key), alpha=0.12)
        out[key] = dict(speeds=speeds, rate=rate, strikes=strike)
    ax.set(xlabel="mean wind speed (m/s), with gusts", ylabel="catch rate (%)", ylim=(-3, 103),
           title="Crosswind: where the catch stops working")
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
    keys = ctl_keys(rows)
    fig, ax = plt.subplots(figsize=(8.5, 1.0 + 0.55 * len(keys)))
    for i, key in enumerate(keys):
        for d in rows:
            if d["controller"] != key or d["engine_out_time"] is None:
                continue
            ax.scatter(d["engine_out_time"], i, s=60, marker="s", color=OC_COL[outcome(d)], edgecolor="white")
    ax.axvline(NOMINAL.switch_time, color="k", ls=":", lw=0.8)
    ax.text(NOMINAL.switch_time + 0.1, len(keys) - 0.45, "13 -> 3 engines", fontsize=8)
    ax.set_yticks(range(len(keys)), [label(k) for k in keys])
    ax.set(xlabel="time of the engine failure (s after ignition)", ylim=(-0.6, len(keys) - 0.3),
           title="One engine fails: when can the catch still be saved?")
    ax.grid(False)
    ax.legend(handles=[mpatches.Patch(color=c, label=o) for o, c in OC_COL.items()], loc="upper center",
              bbox_to_anchor=(0.5, -0.35), ncol=3, fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "engineout.png")
    plt.close(fig)


# ---------------------------------------------------------------- 6. capture basin
def fig_basin():
    rows = load("mc_basin.jsonl")
    if not rows:
        return None
    keys = ctl_keys(rows)
    fig, axs = plt.subplots(1, len(keys), figsize=(5 * len(keys), 4.4), sharey=True)
    out = {}
    for ax, key in zip(np.atleast_1d(axs), keys):
        rs = [d for d in rows if d["controller"] == key]
        for d in rs:
            o = outcome(d)
            ax.add_patch(mpatches.Rectangle((d["dx"] - 18, d["dz"] - 22.5), 36, 45, color=OC_COL[o], alpha=0.85, lw=0))
            if d["caught"]:
                ax.text(d["dx"], d["dz"], f"{d['propellant_left'] / 1e3:.1f}", ha="center", va="center", fontsize=6,
                        color="white")
        ax.plot(0, 0, "k+", ms=10)
        ax.set(xlim=(-220, 220), ylim=(-325, 325), xlabel="ignition point shifted towards the tower (m)",
               title=f"{label(key)}: {sum(d['caught'] for d in rs)}/{len(rs)} caught")
        ax.grid(False)
        out[key] = (sum(d["caught"] for d in rs), len(rs), sum(outcome(d) == "pad strike" for d in rs))
    np.atleast_1d(axs)[0].set_ylabel("ignition altitude shift (m)")
    fig.suptitle("Capture basin: from which ignition points can each controller still catch the booster?\n"
                 "(numbers = tonnes of propellant left; calm air, nominal vehicle)", fontsize=10)
    fig.legend(handles=[mpatches.Patch(color=c, label=o) for o, c in OC_COL.items()], loc="lower center", ncol=3,
               fontsize=8, frameon=False)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(OUT / "basin.png")
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
    print("figures in", OUT)
