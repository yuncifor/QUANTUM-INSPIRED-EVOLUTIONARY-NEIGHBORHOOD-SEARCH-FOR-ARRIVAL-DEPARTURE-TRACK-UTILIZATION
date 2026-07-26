# QUANTUM-INSPIRED-EVOLUTIONARY-NEIGHBORHOOD-SEARCH-FOR-ARRIVAL-DEPARTURE-TRACK-UTILIZATION 
# QEA-NS railway disruption benchmark

This repository contains the minimum runnable code and input data for a
railway passenger-station disruption recovery benchmark. The benchmark
compares QEA-NS with CP-SAT under shared candidate resources and feasibility
constraints.

## Requirements

- Python 3.9 or newer
- Dependencies listed in `requirements.txt`

Install the dependencies with:

```bash
python -m pip install -r requirements.txt
```

## Main benchmark

Run from the repository root:

```bash
python run_frankfurt_hbf_benchmark.py   --input data/frankfurt_hbf_gtfs_schedule.csv   --output-dir results/benchmark   --seed 42   --source-train-count 3   --max-route-candidates 5   --qea-time-limit 400   --subproblem-time-limit 120   --cp-sat-time-limit 400   --qea-pop-size 50   --qea-max-generations 500   --short-case-train-limit 0
```

The multi-seed and source-count sensitivity runners expose their complete
options through `--help` and use the same benchmark entry point.

## Data

`data/frankfurt_hbf_gtfs_schedule.csv` is a GTFS-derived schedule snapshot
used as the benchmark input. Check the source data license and attribution
requirements before redistributing a modified copy or publishing derived
datasets.

## Scope

Generated result directories, figures, paper documents, local environment
files, and private project metadata are intentionally excluded from this
minimal release.
