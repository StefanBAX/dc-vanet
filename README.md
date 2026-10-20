# Urban VANET — beacon reliability and multi-hop alert delivery (SUMO + ns-3)

Data Communications group project (UPB ETTI, 2026–2027), Project 4.
Team: Ianna, Ciprian, Ștefan.

```
SUMO (1 km × 1 km grid, 3×3 traffic lights)  ──fcd.xml──►  traceExporter.py  ──mobility.tcl──►  ns-3.48
                                                                                 802.11p 10 MHz, ch. 178
                                                                                 beacons 10 Hz / 300 B broadcast
                                                                                 alerts 500 B → RSU via AODV | OLSR
```

Everything runs in simulated time (one clock); SUMO is coupled offline through exported traces.

## Reproduce the main results (one command)

```bash
pip install -r requirements.txt
export NS3_DIR=~/ns-3.48          # an ns-3.48 tree, built once (below)
./run_all.sh                      # 35 runs → results/summary.csv and results/plots/*.png
```

On a 2-core laptop the matrix takes about 1 h (120-vehicle runs ≈ 4 min each, 60 ≈ 1 min, 30 ≈ 10 s)
and writes about 1.3 GB of logs; `--jobs N` runs N simulations in parallel.

```bash
```

## One-time setup

```bash
sudo apt install g++ cmake ninja-build git python3-pip
git clone --depth 1 --branch ns-3.48 https://gitlab.com/nsnam/ns-3-dev.git ~/ns-3.48
cd ~/ns-3.48
./ns3 configure -d optimized --enable-modules="wifi;mobility;internet;applications;aodv;olsr;flow-monitor" \
    --disable-examples --disable-tests
./ns3 build                       # 30–60 min the first time
```

The runner copies `ns3/vanet.cc` into `$NS3_DIR/scratch/` and rebuilds it when it changes.

## Step by step

```bash
python3 sumo/make_mobility.py --vehicles 30 --seeds 1001     # SUMO trace only
python3 scripts/two_node_test.py                             # P0: 2 parked cars at 50–600 m
python3 scripts/run_matrix.py --smoke                        # 30 vehicles, AODV, 40 s
cat data/runs/smoke_*/run_info.txt                           # P0 check: channel 178, 10 MHz, 5890 MHz
python3 scripts/run_matrix.py --midterm                      # 30 vehicles, AODV, 120 s
python3 analysis/plots.py --midterm data/runs/midterm_n30_aodv_b10_s1001
```

## Scenario

| Item | Value |
|---|---|
| Road network | SUMO `netgenerate` grid, junctions at 0/500/1000 m, 1 lane per direction, 50 km/h, all 9 junctions signalised |
| Vehicles | {30, 60, 120}, all inserted in the first 10 s, each route with 6 intermediate waypoints so nobody leaves before 120 s |
| RSU | static node at the central intersection (500, 500) |
| Radio | IEEE 802.11p, channel 178 (5.89 GHz, 10 MHz), ad-hoc MAC, 6 Mbit/s, 20 dBm, log-distance loss (exponent 2.5, 47.86 dB at 1 m), no buildings; preamble detection from −94 dBm (ns-3's −82 dBm default is the 20 MHz 802.11a level and would cut the range to ~150 m) → single-hop range ≈ 420 m |
| Beacons | 300 B UDP broadcast, 10 Hz (2 Hz for the rate point), random phase per vehicle |
| Alerts | 500 B UDP unicast to the RSU, one random vehicle every 5 s, routed by AODV or OLSR |
| Time | 120 s simulated, first 20 s discarded (routing and MAC warm-up) |
| Seeds | 1001–1005: one SUMO route file per (vehicles, seed), reused for AODV, OLSR and 2 Hz; ns-3 RNG run = seed index |

Matrix: vehicles {30, 60, 120} × routing {AODV, OLSR} × 5 seeds = 30 runs, plus 120 vehicles × AODV × 2 Hz × 5 seeds = 5 runs.

## Outputs of one run (`data/runs/<run_id>/`)

| File | Content |
|---|---|
| `beacon_tx.csv` | every beacon: time, sender, seq, position, vehicles within 0–100 / 100–200 / 200–300 m |
| `beacon_rx.csv` | every beacon reception: time, receiver, sender, seq, distance |
| `alert_tx.csv`, `alert_rx.csv` | every alert sent / received at the RSU (with TTL and hop count) |
| `cbr.csv` | per node: time the PHY was TX, RX or CCA_BUSY after the warm-up |
| `routing.csv` | per node: AODV (UDP 654) or OLSR (UDP 698) control packets and bytes sent |
| `run_info.txt` | channel, width and frequency the PHY actually used |
| `kpi.json`, `alert_delays.csv` | KPIs computed by `analysis/kpi.py` |

## KPI definitions

| KPI | Definition |
|---|---|
| Beacon PDR (bin) | receptions in the distance bin / Σ over beacons of the vehicles in that bin at transmission |
| Channel busy ratio | (TX + RX + CCA_BUSY time) / 100 s, mean over vehicles |
| Alert delivery ratio | alerts received by the RSU / alerts sent |
| Alert delay | t_rx − t_tx (mean, p95, CDF, fraction above 100 ms) |
| Routing overhead | control packets sent by all nodes / alerts delivered |
| Hops per alert | 64 − TTL at the RSU + 1 |

Per matrix point: mean over 5 seeds and 95 % CI, `t(0.975, 4)·s/√5`.

## Repository layout and ownership

| Path | What | Owner |
|---|---|---|
| `sumo/` | road network, routes, SUMO run, ns-2 trace export | Ciprian |
| `ns3/vanet.cc` | 802.11p setup, beacon and alert applications, routing, traces | Ianna |
| `scripts/`, `analysis/`, `run_all.sh` | matrix runner, KPIs, statistics, figures | Ștefan |

## Pitfalls we handle

- The ns-3 `wave` module is gone since ns-3.39: we use `WIFI_STANDARD_80211p` with `AdhocWifiMac`, and `run_info.txt` proves the 10 MHz channel.
- Broadcast frames have no MAC retries: beacon PDR at 200–300 m reflects path loss and collisions; we explain it, we do not tune it away.
- The same route file is used for AODV and OLSR in every seed.
