#!/usr/bin/python3.10
"""Summarize per-case batch JSON results."""
import argparse
import csv
import json
import pathlib
import sys
from statistics import mean


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()

    results_dir = pathlib.Path(args.results_dir)
    records = []
    for path in sorted(results_dir.glob("*.json")):
        if path.name == "summary.json":
            continue
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if "case_id" in payload:
            records.append(payload)

    if not records:
        print("No per-case result json files found.", file=sys.stderr)
        return 1

    success_count = sum(1 for record in records if record.get("success"))
    summary = {
        "total_cases": len(records),
        "success_count": success_count,
        "failure_count": len(records) - success_count,
        "success_rate": round(success_count / len(records), 4),
        "mean_elapsed_sec": round(mean(record["elapsed_sec"] for record in records), 3),
        "max_elapsed_sec": max(record["elapsed_sec"] for record in records),
        "min_elapsed_sec": min(record["elapsed_sec"] for record in records),
        "cases": [
            {
                "case_id": record["case_id"],
                "success": record.get("success", False),
                "elapsed_sec": record.get("elapsed_sec"),
                "nav2_ready": record.get("nav2_ready", False),
            }
            for record in records
        ],
    }

    with open(results_dir / "summary.json", "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    with open(results_dir / "summary.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["case_id", "success", "elapsed_sec", "nav2_ready", "log_file"],
        )
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    "case_id": record["case_id"],
                    "success": record.get("success", False),
                    "elapsed_sec": record.get("elapsed_sec"),
                    "nav2_ready": record.get("nav2_ready", False),
                    "log_file": record.get("log_file", ""),
                }
            )

    print(json.dumps(summary, indent=2))
    return 0 if success_count == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
