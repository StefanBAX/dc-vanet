"""Every figure of the report, drawn from the logs (results/summary.csv and the run folders).

  python analysis/plots.py                    -> core figures in results/plots/
  python analysis/plots.py --midterm RUN_DIR  -> beacon PDR vs distance + CBR of one run
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "results" / "plots"
BIN_LABELS = ["0–100 m", "100–200 m", "200–300 m"]
BIN_KEYS = ["pdr_0_100", "pdr_100_200", "pdr_200_300"]

# fixed colour + marker per routing protocol, and per density (identity never by colour alone)
ROUTING = {"aodv": dict(color="#2a78d6", marker="o", label="AODV"),
           "olsr": dict(color="#eb6834", marker="s", label="OLSR")}
DENSITY = {30: dict(color="#2a78d6", marker="o"), 60: dict(color="#1baf7a", marker="^"),
           120: dict(color="#eb6834", marker="s")}

plt.rcParams.update({
    "figure.dpi": 150, "font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": "#e3e3e0", "grid.linewidth": 0.6, "lines.linewidth": 2,
    "lines.markersize": 6, "axes.edgecolor": "#888888", "legend.frameon": False,
})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name)
    plt.close(fig)
    print("wrote", OUT / name)


def err(df, k):
    return df[f"{k}_ci95"] if f"{k}_ci95" in df else None


def beacon_pdr_vs_distance(s):
    s10 = s[s.beacon_hz == 10]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0), sharey=True)
    x = np.arange(len(BIN_KEYS))
    for ax, (r, rs) in zip(axes, ROUTING.items()):
        d = s10[s10.routing == r]
        for n, st in DENSITY.items():
            row = d[d.n_vehicles == n]
            if row.empty:
                continue
            y = [row[f"{k}_mean"].iloc[0] for k in BIN_KEYS]
            e = [row[f"{k}_ci95"].iloc[0] for k in BIN_KEYS]
            ax.errorbar(x, y, yerr=e, capsize=3, label=f"{n} vehicles", **st)
        ax.set_xticks(x, BIN_LABELS)
        ax.set_title(f"beacons, runs with {rs['label']}")
        ax.set_xlabel("sender–receiver distance")
    axes[0].set_ylabel("beacon PDR (%)")
    axes[0].set_ylim(0, 105)
    axes[1].legend()
    save(fig, "beacon_pdr_distance.png")


def per_density(s, kpi, ylabel, fname, logy=False):
    s10 = s[s.beacon_hz == 10]
    fig, ax = plt.subplots(figsize=(4.2, 3.0))
    for r, st in ROUTING.items():
        d = s10[s10.routing == r].sort_values("n_vehicles")
        if d.empty or f"{kpi}_mean" not in d:
            continue
        ax.errorbar(d.n_vehicles, d[f"{kpi}_mean"], yerr=err(d, kpi), capsize=3, **st)
    ax.set_xlabel("vehicles")
    ax.set_xticks(sorted(s10.n_vehicles.unique()))
    ax.set_ylabel(ylabel)
    if logy:
        ax.set_yscale("log")
    ax.legend()
    save(fig, fname)


def alert_delay_cdf(runs_dir):
    files = sorted(runs_dir.glob("core_*_b10_*/alert_delays.csv"))
    if not files:
        return
    data = {}
    for f in files:
        meta = json.loads((f.parent / "meta.json").read_text())
        data.setdefault((meta["n_vehicles"], meta["routing"]), []).append(pd.read_csv(f).delay_ms)
    dens = sorted({n for n, _ in data})
    fig, axes = plt.subplots(1, len(dens), figsize=(2.6 * len(dens), 2.8), sharey=True, squeeze=False)
    for ax, n in zip(axes[0], dens):
        for r, st in ROUTING.items():
            if (n, r) not in data:
                continue
            v = np.sort(pd.concat(data[(n, r)]).to_numpy())
            ax.step(v, np.arange(1, len(v) + 1) / len(v), where="post", color=st["color"], label=st["label"])
        ax.axvline(100, color="#888888", linewidth=1, linestyle="--")
        ax.set_xscale("log")
        ax.set_title(f"{n} vehicles")
        ax.set_xlabel("alert delay (ms)")
    axes[0][0].set_ylabel("CDF of delivered alerts")
    axes[0][-1].legend(loc="lower right")
    save(fig, "alert_delay_cdf.png")


def beacon_rate(s):
    d = s[(s.n_vehicles == 120) & (s.routing == "aodv")].set_index("beacon_hz")
    if not {2.0, 10.0} <= set(d.index):
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 3.0), gridspec_kw={"width_ratios": [3, 1.3]})
    x = np.arange(len(BIN_KEYS))
    for off, hz, color in ((-0.18, 10.0, "#eb6834"), (0.18, 2.0, "#2a78d6")):
        y = [d.loc[hz, f"{k}_mean"] for k in BIN_KEYS]
        e = [d.loc[hz, f"{k}_ci95"] for k in BIN_KEYS]
        ax1.bar(x + off, y, width=0.34, yerr=e, capsize=3, color=color, label=f"{hz:.0f} Hz")
    ax1.set_xticks(x, BIN_LABELS)
    ax1.grid(axis="x", visible=False)
    ax1.set_ylabel("beacon PDR (%)")
    ax1.set_ylim(0, 105)
    ax1.set_title("120 vehicles, AODV")
    ax1.legend()
    cb = [d.loc[10.0, "cbr_mean_pct_mean"], d.loc[2.0, "cbr_mean_pct_mean"]]
    ce = [d.loc[10.0, "cbr_mean_pct_ci95"], d.loc[2.0, "cbr_mean_pct_ci95"]]
    ax2.bar([0, 1], cb, yerr=ce, capsize=3, color=["#eb6834", "#2a78d6"], width=0.6)
    ax2.set_xticks([0, 1], ["10 Hz", "2 Hz"])
    ax2.set_ylabel("channel busy ratio (%)")
    save(fig, "beacon_rate_effect.png")


def midterm(run_dir: Path):
    run_dir = Path(run_dir)
    k = json.loads((run_dir / "kpi.json").read_text())
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 3.0), gridspec_kw={"width_ratios": [3, 1.2]})
    y = [k[b] for b in BIN_KEYS]
    bars = ax1.bar(BIN_LABELS, y, color="#2a78d6", width=0.6)
    ax1.bar_label(bars, fmt="%.1f %%", fontsize=8)
    ax1.grid(axis="x", visible=False)
    ax1.set_ylim(0, 110)
    ax1.set_ylabel("beacon PDR (%)")
    ax1.set_title(f"{k['n_vehicles']} vehicles, {k['beacon_hz']:.0f} Hz, {k['beacons_sent']} beacons")
    cbr = pd.read_csv(run_dir / "cbr.csv")
    cbr = cbr[cbr.node < k["n_vehicles"]]
    ax2.boxplot(100 * cbr.cbr, widths=0.5)
    ax2.set_xticks([1], ["vehicles"])
    ax2.set_ylabel("channel busy ratio (%)")
    ax2.set_title(f"mean {k['cbr_mean_pct']:.1f} %")
    save(fig, "midterm_beacons.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--midterm", metavar="RUN_DIR")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if a.midterm:
        midterm(a.midterm)
    else:
        s = pd.read_csv(ROOT / "results" / "summary.csv")
        beacon_pdr_vs_distance(s)
        per_density(s, "cbr_mean_pct", "channel busy ratio (%)", "cbr.png")
        per_density(s, "alert_pdr_pct", "alert delivery ratio (%)", "alert_pdr.png")
        per_density(s, "alert_delay_p95_ms", "alert delay p95 (ms)", "alert_delay_p95.png", logy=True)
        per_density(s, "alert_delay_mean_ms", "alert delay mean (ms)", "alert_delay_mean.png", logy=True)
        per_density(s, "routing_overhead", "control packets / delivered alert", "routing_overhead.png", logy=True)
        per_density(s, "alert_hops_mean", "hops per alert", "alert_hops.png")
        alert_delay_cdf(ROOT / "data" / "runs")
        beacon_rate(s)
