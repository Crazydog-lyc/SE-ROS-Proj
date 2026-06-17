#!/usr/bin/python3.10
"""Batch runner: generate_cases -> per-case launch -> results JSON/CSV."""
import argparse
import csv
import json
import os
import pathlib
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, TextIO

from ament_index_python.packages import get_package_share_directory

MISSION_SUCCEEDED = re.compile(r"Mission succeeded:")
MISSION_FAILED = re.compile(r"Mission failed:")
NAV2_READY = re.compile(r"Ready for navigation!")
DEFAULT_SPAWN_XY = (-2.0, 0.0)


def default_nav2_params_file() -> str:
    share = pathlib.Path(get_package_share_directory("sam_bot_nav2_gz"))
    return str(share / "config" / "nav2_params.yaml")


def ensure_nav2_params_arg(extra_launch_args: List[str]) -> List[str]:
    if any(arg.startswith("nav2_params_file:=") for arg in extra_launch_args):
        return extra_launch_args
    force_base = os.environ.get("FORCE_BASE_NAV2_PARAMS", "").strip().lower()
    if force_base in {"1", "true", "yes", "on"}:
        return extra_launch_args + [f"nav2_params_file:={default_nav2_params_file()}"]
    # Leave nav2_params_file empty so scenario_case.launch.py can use the
    # generated per-case semantic overlay when it exists.
    return extra_launch_args


def cleanup_sim_processes() -> None:
    """Kill leftover sim/launch processes from previous cases."""
    own_pid = os.getpid()
    patterns = [
        r"ign gazebo",
        r"gz sim",
        r"ruby.*ign gazebo",
        r"ros2 launch nav2_scenario_runner run_single_case\.launch",
        r"ros2 launch course_bringup scenario_case\.launch",
        r"ros2 launch sam_bot_nav2_gz complete_navigation\.launch",
        r"rviz2.*navigation_config\.rviz",
    ]
    pids = set()
    for pattern in patterns:
        probe = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True,
            text=True,
        )
        if probe.returncode != 0:
            continue
        for pid_text in probe.stdout.splitlines():
            pid_text = pid_text.strip()
            if not pid_text.isdigit():
                continue
            pid = int(pid_text)
            if pid == own_pid or pid == os.getppid():
                continue
            pids.add(pid)

    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in list(pids):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pids.discard(pid)
            except PermissionError:
                pids.discard(pid)
        if sig == signal.SIGTERM:
            time.sleep(3.0)
    time.sleep(2.0)


def terminate_process_group(proc: subprocess.Popen[Any]) -> int:
    """Stop a ros2 launch process and every child it started."""
    if proc.poll() is not None:
        return int(proc.returncode)

    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None

    for sig, wait_sec in (
        (signal.SIGINT, 30),
        (signal.SIGTERM, 10),
        (signal.SIGKILL, 5),
    ):
        if proc.poll() is not None:
            return int(proc.returncode)
        try:
            if pgid is not None:
                os.killpg(pgid, sig)
            else:
                proc.send_signal(sig)
        except ProcessLookupError:
            break
        try:
            return int(proc.wait(timeout=wait_sec))
        except subprocess.TimeoutExpired:
            continue

    return int(proc.poll() if proc.poll() is not None else -signal.SIGKILL)


def load_manifest_cases(cases_dir: pathlib.Path) -> List[str]:
    manifest = cases_dir / "manifest.csv"
    if not manifest.exists():
        return []
    case_ids: List[str] = []
    with open(manifest, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("success", "").lower() in {"true", "1", "yes"}:
                case_ids.append(row["case_id"])
    return case_ids


def iter_case_files(
    cases_dir: pathlib.Path,
    case_filter: Optional[Iterable[str]] = None,
) -> List[pathlib.Path]:
    allowed = set(case_filter) if case_filter else None
    manifest_ids = load_manifest_cases(cases_dir)
    if manifest_ids:
        files = [
            cases_dir / f"{case_id}_scenario.yaml"
            for case_id in manifest_ids
            if (cases_dir / f"{case_id}_scenario.yaml").exists()
        ]
    else:
        files = sorted(cases_dir.glob("*_scenario.yaml"))

    if allowed:
        files = [path for path in files if path.stem.replace("_scenario", "") in allowed]
    return files


def parse_log_summary(log_path: pathlib.Path) -> Dict[str, Any]:
    summary: Dict[str, Any] = {
        "mission_succeeded": False,
        "mission_failed": False,
        "nav2_ready": False,
        "mission_message": "",
    }
    if not log_path.exists():
        return summary

    try:
        text = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return summary

    if NAV2_READY.search(text):
        summary["nav2_ready"] = True
    for line in text.splitlines():
        if MISSION_SUCCEEDED.search(line):
            summary["mission_succeeded"] = True
            summary["mission_message"] = line.strip()
            break
        if MISSION_FAILED.search(line):
            summary["mission_failed"] = True
            summary["mission_message"] = line.strip()
            break
    return summary


def _pipe_to_log_and_stream(
    pipe: Any,
    logf: TextIO,
    stream: TextIO,
    *,
    prefix: str = "",
) -> None:
    try:
        for line in iter(pipe.readline, ""):
            if not line:
                break
            logf.write(f"{prefix}{line}")
            logf.flush()
            stream.write(line)
            stream.flush()
    finally:
        pipe.close()


def run_case(
    case_file: pathlib.Path,
    results_dir: pathlib.Path,
    launch_package: str,
    launch_file: str,
    timeout_sec: int,
    extra_launch_args: List[str],
    tee_output: bool,
    ros_domain_id: Optional[int],
) -> Dict[str, Any]:
    case_id = case_file.stem.replace("_scenario", "")
    cleanup_sim_processes()

    cmd = [
        "ros2",
        "launch",
        launch_package,
        launch_file,
        f"scenario_file:={case_file}",
    ] + extra_launch_args

    log_path = results_dir / f"{case_id}.log"
    env = os.environ.copy()
    env["PATH"] = "/usr/bin:/opt/ros/humble/bin:" + env.get("PATH", "")
    if ros_domain_id is not None:
        env["ROS_DOMAIN_ID"] = str(ros_domain_id)

    start = time.time()
    with open(log_path, "w", encoding="utf-8", buffering=1) as logf:
        logf.write(f"=== case {case_id} ===\n")
        logf.write(f"cmd: {' '.join(cmd)}\n\n")
        if ros_domain_id is not None:
            logf.write(f"ROS_DOMAIN_ID={ros_domain_id}\n\n")
        logf.flush()

        popen_kwargs: Dict[str, Any] = {"env": env}
        if tee_output:
            popen_kwargs.update(
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        else:
            popen_kwargs.update(stdout=logf, stderr=subprocess.STDOUT)

        proc = subprocess.Popen(cmd, start_new_session=True, **popen_kwargs)
        stream_threads: List[threading.Thread] = []
        if tee_output and proc.stdout is not None and proc.stderr is not None:
            stream_threads = [
                threading.Thread(
                    target=_pipe_to_log_and_stream,
                    args=(proc.stdout, logf, sys.stdout),
                    daemon=True,
                ),
                threading.Thread(
                    target=_pipe_to_log_and_stream,
                    args=(proc.stderr, logf, sys.stderr),
                    kwargs={"prefix": "[stderr] "},
                    daemon=True,
                ),
            ]
            for thread in stream_threads:
                thread.start()

        timed_out = False
        try:
            rc = proc.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            timed_out = True
            rc = terminate_process_group(proc)
        except KeyboardInterrupt:
            terminate_process_group(proc)
            cleanup_sim_processes()
            raise
        finally:
            for thread in stream_threads:
                thread.join(timeout=5)

    if proc.poll() is None:
        rc = terminate_process_group(proc)
    cleanup_sim_processes()
    elapsed = time.time() - start
    log_summary = parse_log_summary(log_path)
    success = (not timed_out) and log_summary["mission_succeeded"] and not log_summary["mission_failed"]

    result = {
        "case_id": case_id,
        "scenario_file": str(case_file),
        "return_code": rc,
        "success": success,
        "timed_out": timed_out,
        "elapsed_sec": round(elapsed, 3),
        "ros_domain_id": ros_domain_id,
        "log_file": str(log_path),
        **log_summary,
    }
    with open(results_dir / f"{case_id}.json", "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Run generated scenario cases sequentially.")
    parser.add_argument("--cases-dir", required=True)
    parser.add_argument("--results-dir", required=True)
    parser.add_argument("--launch-package", default="nav2_scenario_runner")
    parser.add_argument("--launch-file", default="run_single_case.launch.py")
    parser.add_argument("--timeout-sec", type=int, default=900)
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--extra-launch-arg", action="append", default=[])
    parser.add_argument(
        "--ros-domain-base",
        type=int,
        default=None,
        help="If set, run each case in an isolated ROS_DOMAIN_ID starting here.",
    )
    parser.add_argument(
        "--tee-output",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Mirror subprocess stdout/stderr to terminal while writing logs (default: on).",
    )
    args = parser.parse_args()

    cases_dir = pathlib.Path(args.cases_dir)
    results_dir = pathlib.Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    extra_launch_args = ensure_nav2_params_arg(args.extra_launch_arg)
    case_files = iter_case_files(cases_dir, args.case_id or None)
    if not case_files:
        print(f"No scenario files found under {cases_dir}", file=sys.stderr)
        return 1

    rows: List[Dict[str, Any]] = []
    for index, case_file in enumerate(case_files, start=1):
        case_id = case_file.stem.replace("_scenario", "")
        ros_domain_id = None
        if args.ros_domain_base is not None:
            ros_domain_id = (int(args.ros_domain_base) + index - 1) % 233
        print(f"[{index}/{len(case_files)}] Running {case_id}")
        row = run_case(
            case_file,
            results_dir,
            args.launch_package,
            args.launch_file,
            args.timeout_sec,
            extra_launch_args,
            args.tee_output,
            ros_domain_id,
        )
        status = "PASS" if row["success"] else "FAIL"
        print(
            f"  -> {status} rc={row['return_code']} elapsed={row['elapsed_sec']}s "
            f"nav2_ready={row['nav2_ready']}"
        )
        rows.append(row)

    fieldnames = [
        "case_id",
        "return_code",
        "success",
        "timed_out",
        "elapsed_sec",
        "ros_domain_id",
        "nav2_ready",
        "mission_succeeded",
        "mission_failed",
        "log_file",
    ]
    with open(results_dir / "results.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    passed = sum(1 for row in rows if row["success"])
    print(f"Batch finished: {passed}/{len(rows)} passed. Wrote {results_dir / 'results.csv'}")
    return 0 if passed == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
