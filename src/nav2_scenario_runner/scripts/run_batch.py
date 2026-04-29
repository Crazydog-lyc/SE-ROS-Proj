#!/usr/bin/env python3
import argparse
import csv
import json
import pathlib
import subprocess
import time
from typing import Any, Dict, List

def run_case(case_file: pathlib.Path, results_dir: pathlib.Path, launch_package: str, launch_file: str,
             timeout_sec: int, extra_launch_args: List[str]) -> Dict[str, Any]:
    case_id = case_file.stem.replace("_scenario", "")
    start = time.time()
    cmd = ["ros2", "launch", launch_package, launch_file, f"scenario_file:={case_file}"] + extra_launch_args

    log_path = results_dir / f"{case_id}.log"
    with open(log_path, "w", encoding="utf-8") as logf:
        proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
        timed_out = False
        try:
            rc = proc.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            timed_out = True
            proc.terminate()
            try:
                rc = proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                rc = proc.wait()

    elapsed = time.time() - start
    result = {
        "case_id": case_id,
        "return_code": rc,
        "success": rc == 0 and not timed_out,
        "timed_out": timed_out,
        "elapsed_sec": round(elapsed, 3),
        "log_file": str(log_path),
    }
    with open(results_dir / f"{case_id}.json", "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    return result

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases-dir", required=True)
    parser.add_argument("--results-dir", required=True)
    parser.add_argument("--launch-package", required=True)
    parser.add_argument("--launch-file", required=True)
    parser.add_argument("--timeout-sec", type=int, default=180)
    parser.add_argument("--extra-launch-arg", action="append", default=[])
    args = parser.parse_args()

    cases_dir = pathlib.Path(args.cases_dir)
    results_dir = pathlib.Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for case_file in sorted(cases_dir.glob("*_scenario.yaml")):
        print(f"Running {case_file.name}")
        rows.append(run_case(case_file, results_dir, args.launch_package, args.launch_file,
                             args.timeout_sec, args.extra_launch_arg))

    with open(results_dir / "results.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["case_id", "return_code", "success", "timed_out", "elapsed_sec", "log_file"])
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {results_dir / 'results.csv'}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
