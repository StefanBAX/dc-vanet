"""KPIs of one ns-3 run, computed only from its logs (data/runs/<run_id>/).

Every log already starts at the end of the warm-up (20 s), so the measurement
window is [20 s, 120 s) = 100 s.

Definitions (used as-is in the report):
  Beacon PDR per distance bin b in {0-100, 100-200, 200-300 m}
      = receptions with sender-receiver distance in b / sum over all beacons of the
        vehicles that were in b when the beacon was sent        (x 100 %)
  Channel busy ratio (CBR) = time the PHY was in TX, RX or CCA_BUSY / window,
      averaged over the vehicles (RSU excluded); max over vehicles also reported
  Alert delivery ratio = alerts received by the RSU / alerts sent (x 100 %)
  Alert delay = t_rx - t_tx at the RSU (ms; mean, p95, fraction above 100 ms)
  Routing overhead = AODV/OLSR control packets sent by all nodes / alerts delivered
  Hops per alert = 64 - TTL at the RSU + 1 (mean over delivered alerts)

Usage: python analysis/kpi.py data/runs/<run_id> [...]
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

BINS = [(0, 100), (100, 200), (200, 300)]


def compute(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "meta.json").read_text())
    window = meta["sim_time_s"] - meta["warmup_s"]
    n_veh = meta["n_vehicles"]

    btx = pd.read_csv(run_dir / "beacon_tx.csv")
    brx = pd.read_csv(run_dir / "beacon_rx.csv")
    atx = pd.read_csv(run_dir / "alert_tx.csv")
    arx = pd.read_csv(run_dir / "alert_rx.csv")
    cbr = pd.read_csv(run_dir / "cbr.csv")
    rt = pd.read_csv(run_dir / "routing.csv")

    k = {key: meta[key] for key in ("run_id", "n_vehicles", "routing", "beacon_hz", "seed")}
    k.update({"window_s": window,
              "beacons_sent": len(btx), "beacon_receptions": len(brx),
              "beacon_offered_kbps": len(btx) * meta["beacon_size_b"] * 8 / window / 1000})

    # ---- beacons: PDR per distance bin
    expected_cols = ["nb100", "nb200", "nb300"]
    for (lo, hi), col in zip(BINS, expected_cols):
        expected = int(btx[col].sum())
        received = int(((brx.dist_m >= lo) & (brx.dist_m < hi)).sum())
        k[f"pdr_{lo}_{hi}"] = 100.0 * received / expected if expected else None
        k[f"expected_{lo}_{hi}"] = expected
        k[f"received_{lo}_{hi}"] = received
    exp_all = int(btx[expected_cols].sum().sum())
    rec_all = int((brx.dist_m < 300).sum())
    k["pdr_0_300"] = 100.0 * rec_all / exp_all if exp_all else None
    k["neighbours_300m"] = float(btx[expected_cols].sum(axis=1).mean()) if len(btx) else None

    # ---- channel busy ratio (vehicles only; the RSU has node id n_veh)
    veh = cbr[cbr.node < n_veh]
    k["cbr_mean_pct"] = 100.0 * float(veh.cbr.mean())
    k["cbr_max_pct"] = 100.0 * float(veh.cbr.max())

    # ---- alerts to the RSU
    first = arx.sort_values("t_ns").drop_duplicates("id", keep="first")
    joined = first.merge(atx[["id", "t_ns", "dist_rsu_m"]], on="id", suffixes=("_rx", "_tx"))
    delay_ms = (joined.t_ns_rx - joined.t_ns_tx).to_numpy() / 1e6
    k["alerts_sent"] = len(atx)
    k["alerts_delivered"] = len(first)
    k["alert_pdr_pct"] = 100.0 * len(first) / len(atx) if len(atx) else None
    k["alert_delay_mean_ms"] = float(delay_ms.mean()) if len(delay_ms) else None
    k["alert_delay_p50_ms"] = float(np.percentile(delay_ms, 50)) if len(delay_ms) else None
    k["alert_delay_p95_ms"] = float(np.percentile(delay_ms, 95)) if len(delay_ms) else None
    k["alert_frac_over_100ms"] = float((delay_ms > 100).mean()) if len(delay_ms) else None
    k["alert_hops_mean"] = float(first.hops[first.hops > 0].mean()) if len(first) else None
    k["alert_dist_rsu_mean_m"] = float(atx.dist_rsu_m.mean()) if len(atx) else None

    # ---- routing overhead
    ctrl = int(rt.ctrl_pkts.sum())
    k["routing_ctrl_pkts"] = ctrl
    k["routing_ctrl_kbps"] = int(rt.ctrl_bytes.sum()) * 8 / window / 1000
    k["routing_overhead"] = ctrl / len(first) if len(first) else None

    (run_dir / "kpi.json").write_text(json.dumps(k, indent=2))
    # per-alert delays, kept for the CDF plots
    joined.assign(delay_ms=delay_ms)[["id", "src", "dist_rsu_m", "hops", "delay_ms"]].to_csv(
        run_dir / "alert_delays.csv", index=False)
    return k


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for d in sys.argv[1:]:
        r = compute(Path(d))
        print(f"{r['run_id']}: beacon PDR {r['pdr_0_100']:.1f}/{r['pdr_100_200']:.1f}/{r['pdr_200_300']:.1f} %  "
              f"CBR {r['cbr_mean_pct']:.1f} %  alerts {r['alerts_delivered']}/{r['alerts_sent']}  "
              f"delay p95 {r['alert_delay_p95_ms']}  overhead {r['routing_overhead']}")
