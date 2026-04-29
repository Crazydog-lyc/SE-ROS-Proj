#!/usr/bin/env python3
import argparse
import json
import pathlib
from statistics import mean

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()

    results_dir = pathlib.Path(args.results_dir)
    records = []
    for path in sorted(results_dir.glob("*.json")):
        with open(path, "r", encoding="utf-8") as f:
            records.append(json.load(f))

    if not records:
        print("No result json files found.")
        return 1

    success_count = sum(1 for r in records if r["success"])
    summary = {
        "total_cases": len(records),
        "success_count": success_count,
        "failure_count": len(records) - success_count,
        "success_rate": success_count / len(records),
        "mean_elapsed_sec": mean(r["elapsed_sec"] for r in records),
        "max_elapsed_sec": max(r["elapsed_sec"] for r in records),
        "min_elapsed_sec": min(r["elapsed_sec"] for r in records),
    }

    with open(results_dir / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(json.dumps(summary, indent=2))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
