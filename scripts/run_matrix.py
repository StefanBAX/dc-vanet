#!/usr/bin/env python3
"""Runs the VANET experiment matrix and computes every run's KPIs.

Core matrix: vehicles {30, 60, 120} x routing {aodv, olsr} x 5 seeds          = 30 runs
Beacon-rate point: 120 vehicles, AODV, 2 Hz beacons (instead of 10 Hz) x 5 seeds = 5 runs

The SUMO trace of (vehicles, seed) is shared by AODV, OLSR and the 2 Hz runs, and
the ns-3 RNG run number equals the seed index, so the systems compared see the
same vehicles, beacon phases and alert sources.

Usage:
  export NS3_DIR=~/ns-3.48
  python3 scripts/run_matrix.py                 # all 35 runs, in parallel
  python3 scripts/run_matrix.py --smoke         # 30 vehicles, AODV, 1 seed, 40 s
  python3 scripts/run_matrix.py --midterm       # 30 vehicles, AODV, 1 seed, full length
  python3 scripts/run_matrix.py --jobs 2 --skip-existing
"""
import argparse
import filecmp
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "analysis"))
import kpi  # noqa: E402

SIM_TIME_S = 120.0
WARMUP_S = 20.0
BEACON_SIZE = 300


def ns3_binary(ns3_dir: Path) -> Path:
    """Copies ns3/vanet.cc into scratch/ (if changed), builds it and returns the binary."""
    src = ROOT / "ns3" / "vanet.cc"
    dst = ns3_dir / "scratch" / "vanet.cc"
    if not dst.exists() or not filecmp.cmp(src, dst, shallow=False):
        shutil.copy(src, dst)
    subprocess.run(["./ns3", "build", "vanet"], cwd=ns3_dir, check=True)
    candidates = sorted((ns3_dir / "build" / "scratch").glob("ns3*-vanet*"))
    if not candidates:
        sys.exit("vanet binary not found under build/scratch - did the build succeed?")
    return candidates[-1]


def one_run(binary, ns3_dir, run_id, n, routing, hz, seed, seed_index, sim_time):
    d = ROOT / "data" / "runs" / run_id
    d.mkdir(parents=True, exist_ok=True)
    mob = ROOT / "data" / "mobility" / f"n{n}_s{seed}" / "mobility.tcl"
    args = [str(binary), f"--mobility={mob}", f"--nVeh={n}", f"--routing={routing}",
            f"--run={seed_index}", f"--beaconHz={hz}", f"--beaconSize={BEACON_SIZE}",
            f"--simTime={sim_time}", f"--warmup={WARMUP_S}", f"--outDir={d}"]
    env = dict(os.environ, LD_LIBRARY_PATH=f"{ns3_dir / 'build' / 'lib'}:{os.environ.get('LD_LIBRARY_PATH', '')}")
    t = time.time()
    with open(d / "ns3.log", "w") as log:
        subprocess.run(args, check=True, stdout=log, stderr=subprocess.STDOUT, env=env)
    meta = {"run_id": run_id, "n_vehicles": n, "routing": routing, "beacon_hz": hz, "seed": seed,
            "ns3_run": seed_index, "sim_time_s": sim_time, "warmup_s": WARMUP_S,
            "beacon_size_b": BEACON_SIZE, "wallclock_s": round(time.time() - t, 1),
            "mobility": str(mob.relative_to(ROOT))}
    (d / "meta.json").write_text(json.dumps(meta, indent=2))
    return kpi.compute(d), meta["wallclock_s"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", type=int, nargs="+", default=[30, 60, 120])
    ap.add_argument("--routing", nargs="+", default=["aodv", "olsr"])
    ap.add_argument("--seeds", type=int, nargs="+", default=[1001, 1002, 1003, 1004, 1005])
    ap.add_argument("--no-rate-point", action="store_true", help="skip the 2 Hz / 120 vehicles point")
    ap.add_argument("--sim-time", type=float, default=SIM_TIME_S)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--skip-existing", action="store_true")
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--midterm", action="store_true")
    a = ap.parse_args()
    if a.smoke or a.midterm:
        a.vehicles, a.routing, a.seeds, a.no_rate_point = [30], ["aodv"], [1001], True
    if a.smoke:
        a.sim_time = 40.0

    ns3_dir = Path(os.environ.get("NS3_DIR", "")).expanduser()
    if not (ns3_dir / "ns3").exists():
        sys.exit("set NS3_DIR to your ns-3 folder (the one containing ./ns3)")

    vehicles_needed = sorted(set(a.vehicles + ([] if a.no_rate_point else [120])))
    subprocess.run([sys.executable, str(ROOT / "sumo" / "make_mobility.py"), "--vehicles",
                    *map(str, vehicles_needed), "--seeds", *map(str, a.seeds)], check=True)
    binary = ns3_binary(ns3_dir)

    tag = "smoke" if a.smoke else ("midterm" if a.midterm else "core")
    plan = []
    for idx, seed in enumerate(a.seeds, 1):
        for n in a.vehicles:
            for r in a.routing:
                plan.append((f"{tag}_n{n}_{r}_b10_s{seed}", n, r, 10.0, seed, idx))
        if not a.no_rate_point:
            plan.append((f"{tag}_n120_aodv_b2_s{seed}", 120, "aodv", 2.0, seed, idx))
    if a.skip_existing:
        plan = [p for p in plan if not (ROOT / "data" / "runs" / p[0] / "kpi.json").exists()]
    # slowest runs first, so the parallel pool finishes together
    plan.sort(key=lambda p: (-p[1], -p[3]))
    print(f"{len(plan)} runs, {a.jobs} in parallel", flush=True)

    t0 = time.time()
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        futs = {pool.submit(one_run, binary, ns3_dir, *p, a.sim_time): p[0] for p in plan}
        for i, f in enumerate(as_completed(futs), 1):
            k, wall = f.result()
            print(f"[{i}/{len(plan)}] {futs[f]} ({wall:.0f} s): beacon PDR "
                  f"{k['pdr_0_100']:.1f}/{k['pdr_100_200']:.1f}/{k['pdr_200_300']:.1f} %, "
                  f"CBR {k['cbr_mean_pct']:.1f} %, alerts {k['alerts_delivered']}/{k['alerts_sent']}", flush=True)
    print(f"done in {(time.time() - t0) / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
