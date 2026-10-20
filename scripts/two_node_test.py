#!/usr/bin/env python3
"""P0 check: two parked vehicles at a fixed distance exchange 10 Hz beacons for 30 s.

Proves that the 802.11p setup works (channel 178, 10 MHz, 5890 MHz in run_info.txt)
and that reception fades with distance as the path-loss model predicts.

Usage: NS3_DIR=~/ns-3.48 python3 scripts/two_node_test.py [--distances 50 150 250 350 600]
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
from run_matrix import ns3_binary  # noqa: E402

SIM, WARMUP = 30.0, 5.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distances", type=float, nargs="+", default=[50, 150, 250, 350, 600])
    a = ap.parse_args()
    ns3_dir = Path(os.environ.get("NS3_DIR", "")).expanduser()
    if not (ns3_dir / "ns3").exists():
        sys.exit("set NS3_DIR to your ns-3 folder")
    binary = ns3_binary(ns3_dir)
    env = dict(os.environ, LD_LIBRARY_PATH=str(ns3_dir / "build" / "lib"))

    print(f"{'distance (m)':>12} {'sent':>6} {'received':>9} {'PDR (%)':>8}")
    for d in a.distances:
        out = ROOT / "data" / "twonode" / f"d{int(d)}"
        out.mkdir(parents=True, exist_ok=True)
        # two parked vehicles on the horizontal road y = 500, the RSU stays at (500, 500)
        (out / "mobility.tcl").write_text(
            f"$node_(0) set X_ 100.0\n$node_(0) set Y_ 500.0\n$node_(0) set Z_ 0\n"
            f"$node_(1) set X_ {100.0 + d}\n$node_(1) set Y_ 500.0\n$node_(1) set Z_ 0\n")
        subprocess.run([str(binary), f"--mobility={out / 'mobility.tcl'}", "--nVeh=2", "--routing=aodv",
                        f"--simTime={SIM}", f"--warmup={WARMUP}", f"--outDir={out}"],
                       check=True, env=env, stdout=subprocess.DEVNULL)
        tx = pd.read_csv(out / "beacon_tx.csv")
        rx = pd.read_csv(out / "beacon_rx.csv")
        print(f"{d:>12.0f} {len(tx):>6} {len(rx):>9} {100 * len(rx) / len(tx):>8.1f}")
    print("\n" + (ROOT / "data" / "twonode" / f"d{int(a.distances[0])}" / "run_info.txt").read_text())


if __name__ == "__main__":
    main()
