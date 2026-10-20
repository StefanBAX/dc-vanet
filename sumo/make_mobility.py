#!/usr/bin/env python3
"""SUMO mobility for the VANET scenario: one trace per (vehicle count, seed).

1. netgenerate: 1 km x 1 km Manhattan grid, 3 x 3 signalised intersections
   (junctions at x, y in {0, 500, 1000} m), one lane per direction, 50 km/h.
2. randomTrips.py + duarouter: N vehicles inserted during the first 10 s, each with
   6 intermediate waypoints so it is still driving at the end of the 120 s run
   (the number of vehicles stays ~N for the whole measured window).
3. sumo: 120 s at 0.1 s steps -> fcd.xml (position of every vehicle every 0.1 s).
4. traceExporter.py: fcd.xml -> mobility.tcl (ns-2 format read by Ns2MobilityHelper).

The same mobility.tcl is reused for AODV and OLSR (and for the 2 Hz beacon runs),
so every protocol sees exactly the same vehicles.

Usage: python3 sumo/make_mobility.py --vehicles 30 60 120 --seeds 1001 1002 1003 1004 1005
Output: data/mobility/n<N>_s<seed>/{routes.rou.xml, fcd.xml, mobility.tcl, summary.json}
"""
import argparse
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "mobility"
SIM_END_S = 120
INSERT_WINDOW_S = 10
SPEED_MS = 13.89            # 50 km/h


def sumo_home():
    if "SUMO_HOME" in os.environ:
        return Path(os.environ["SUMO_HOME"])
    try:
        import sumo
        return Path(sumo.SUMO_HOME)
    except ImportError:
        sys.exit("set SUMO_HOME or pip install eclipse-sumo")


def run(cmd, **kw):
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def make_net(tools):
    net = OUT / "grid.net.xml"
    if net.exists():
        return net
    OUT.mkdir(parents=True, exist_ok=True)
    run(["netgenerate", "--grid", "--grid.number", 3, "--grid.length", 500,
         "--default.lanenumber", 1, "--default.speed", SPEED_MS,
         "--default-junction-type", "traffic_light", "--output-file", net], stdout=subprocess.DEVNULL)
    return net


def make_one(tools, net, n, seed):
    d = OUT / f"n{n}_s{seed}"
    if (d / "mobility.tcl").exists():
        return d
    d.mkdir(parents=True, exist_ok=True)
    period = INSERT_WINDOW_S / n
    run([sys.executable, tools / "randomTrips.py", "-n", net, "-o", d / "trips.trips.xml",
         "-r", d / "routes.rou.xml", "--seed", seed, "-b", 0, "-e", f"{INSERT_WINDOW_S - period / 2:.6f}",  # exactly n departures
         "-p", f"{period:.6f}", "--intermediate", 6, "--min-distance", 300, "--validate"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    run(["sumo", "-n", net, "-r", d / "routes.rou.xml", "--begin", 0, "--end", SIM_END_S,
         "--step-length", 0.1, "--seed", seed, "--fcd-output", d / "fcd.xml",
         "--time-to-teleport", 300, "--no-step-log", "true", "--no-warnings", "true"])
    run([sys.executable, tools / "traceExporter.py", "--fcd-input", d / "fcd.xml",
         "--ns2mobility-output", d / "mobility.tcl"], stdout=subprocess.DEVNULL)  # no --shift: keeps the grid at 0..1000 m

    # sanity numbers: vehicles present per second and the coordinate range
    present, xs, ys, ids = {}, [], [], set()
    for _, el in ET.iterparse(d / "fcd.xml"):
        if el.tag == "timestep":
            t = float(el.get("time"))
            if abs(t - round(t)) < 1e-6:
                present[int(round(t))] = len(el)
            for v in el:
                xs.append(float(v.get("x")))
                ys.append(float(v.get("y")))
                ids.add(v.get("id"))
            el.clear()
    summary = {"vehicles_requested": n, "vehicles_seen": len(ids), "seed": seed,
               "present_per_s": present, "x_range": [min(xs), max(xs)], "y_range": [min(ys), max(ys)]}
    (d / "summary.json").write_text(json.dumps(summary, indent=1))
    print(f"n={n} seed={seed}: {len(ids)} vehicles, present at 20 s / 120 s: "
          f"{present.get(20)} / {present.get(119)}", flush=True)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", type=int, nargs="+", default=[30, 60, 120])
    ap.add_argument("--seeds", type=int, nargs="+", default=[1001, 1002, 1003, 1004, 1005])
    a = ap.parse_args()
    tools = sumo_home() / "tools"
    net = make_net(tools)
    for n in a.vehicles:
        for s in a.seeds:
            make_one(tools, net, n, s)


if __name__ == "__main__":
    main()
