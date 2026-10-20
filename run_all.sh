#!/usr/bin/env bash
# One command: SUMO traces, all 35 ns-3 runs, KPIs, statistics and every figure.
# Needs NS3_DIR pointing at an ns-3.48 folder (see README).
set -euo pipefail
cd "$(dirname "$0")"
python3 scripts/run_matrix.py --skip-existing "$@"
python3 analysis/aggregate.py
python3 analysis/plots.py
