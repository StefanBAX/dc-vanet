"""Mean and 95 % confidence interval of every KPI per matrix point.

Point = (n_vehicles, routing, beacon_hz). CI95 = t(0.975, n-1) * s / sqrt(n), n = seeds.
Outputs results/kpi_runs.csv and results/summary.csv.
Usage: python analysis/aggregate.py [data/runs]
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

ROOT = Path(__file__).resolve().parent.parent
POINT = ["n_vehicles", "routing", "beacon_hz"]
KPIS = ["pdr_0_100", "pdr_100_200", "pdr_200_300", "pdr_0_300", "neighbours_300m",
        "cbr_mean_pct", "cbr_max_pct", "beacon_offered_kbps",
        "alert_pdr_pct", "alert_delay_mean_ms", "alert_delay_p50_ms", "alert_delay_p95_ms",
        "alert_frac_over_100ms", "alert_hops_mean", "routing_overhead", "routing_ctrl_kbps"]


def ci95(x):
    x = x.dropna()
    if len(x) < 2:
        return np.nan
    return float(stats.t.ppf(0.975, len(x) - 1) * x.std(ddof=1) / np.sqrt(len(x)))


def main(runs_dir: Path):
    rows = [json.loads(p.read_text()) for p in sorted(runs_dir.glob("core_*/kpi.json"))]
    if not rows:
        sys.exit(f"no core_*/kpi.json under {runs_dir}")
    runs = pd.DataFrame(rows)
    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    runs.to_csv(out / "kpi_runs.csv", index=False)

    summary = []
    for key, g in runs.groupby(POINT):
        rec = dict(zip(POINT, key))
        rec["n_seeds"] = len(g)
        for k in KPIS:
            if k in g and g[k].notna().any():
                rec[f"{k}_mean"] = float(g[k].mean())
                rec[f"{k}_ci95"] = ci95(g[k])
        summary.append(rec)
    summary = pd.DataFrame(summary).sort_values(POINT)
    summary.to_csv(out / "summary.csv", index=False)
    cols = POINT + ["n_seeds", "pdr_0_100_mean", "pdr_200_300_mean", "cbr_mean_pct_mean",
                    "alert_pdr_pct_mean", "alert_delay_p95_ms_mean", "routing_overhead_mean"]
    print(summary[[c for c in cols if c in summary]].to_string(index=False, float_format="%.2f"))


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "data" / "runs")
