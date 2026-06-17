#!/usr/bin/env python3
"""nav2_mission_manager 质量门禁：静态分析 + 单元测试 + 覆盖率报告。

用法:
  bash scripts/run_quality_gate.sh          # 推荐：自动 source ROS + 工作空间
  python3.10 scripts/run_quality_gate.py    # 需已 colcon build；脚本会尝试补全 PYTHONPATH
  python3.10 scripts/run_quality_gate.py --html

门禁阈值（与课程 Bug Track 对齐）:
  - flake8: 0 error（E9/F 类）
  - bandit: 无 high/medium 漏洞（若已安装 bandit）
  - pytest: 全部通过
  - 语句覆盖 >= 90%
  - 分支覆盖 >= 60%
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


PKG_ROOT = Path(__file__).resolve().parents[1]
PYTHON = shutil.which("python3.10") or "python3.10"
REPORT_DIR = PKG_ROOT / "test_reports"
STMT_MIN = 90.0
BRANCH_MIN = 60.0


def find_workspace_root(start: Path) -> Path | None:
    for parent in [start, *start.parents]:
        if (parent / "install" / "setup.bash").is_file():
            return parent
    return None


def build_test_env() -> dict[str, str]:
    """把源码包 + colcon install + ROS Humble 加入 PYTHONPATH。"""
    env = os.environ.copy()
    paths: list[str] = [str(PKG_ROOT)]

    ros_root = Path("/opt/ros/humble")
    for ros_py in (
        ros_root / "lib/python3.10/site-packages",
        ros_root / "local/lib/python3.10/dist-packages",
    ):
        if ros_py.is_dir():
            paths.append(str(ros_py))
    if (ros_root / "lib").is_dir():
        ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = f"{ros_root / 'lib'}:{ld}" if ld else str(ros_root / "lib")
        ament = env.get("AMENT_PREFIX_PATH", "")
        env["AMENT_PREFIX_PATH"] = f"{ros_root}:{ament}" if ament else str(ros_root)

    ws = find_workspace_root(PKG_ROOT)
    if ws is not None:
        install = ws / "install"
        lib_paths: list[str] = []
        for lib_dir in install.glob("*/lib"):
            if lib_dir.is_dir():
                lib_paths.append(str(lib_dir))
        if lib_paths:
            ld = env.get("LD_LIBRARY_PATH", "")
            merged = ":".join(lib_paths + ([ld] if ld else []))
            env["LD_LIBRARY_PATH"] = merged

        for site in install.glob("*/lib/python3.10/site-packages"):
            paths.append(str(site))
        for dist in install.glob("*/local/lib/python3.10/dist-packages"):
            paths.append(str(dist))
        local = install / "local/lib/python3.10/dist-packages"
        if local.is_dir():
            paths.append(str(local))

    existing = env.get("PYTHONPATH", "")
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = ":".join(dict.fromkeys(paths))
    return env


def check_prerequisites(env: dict[str, str]) -> str | None:
    if not shutil.which(PYTHON):
        return f"未找到 {PYTHON}。ROS Humble 测试需 Python 3.10，不能用 conda 的 python3.12。"

    probe = subprocess.run(
        [
            PYTHON,
            "-c",
            "import course_interfaces; import rclpy; "
            "from course_interfaces.msg import SafetyState; SafetyState()",
        ],
        cwd=PKG_ROOT,
        env=env,
        text=True,
        capture_output=True,
    )
    if probe.returncode != 0:
        return (
            "找不到 course_interfaces / rclpy。\n"
            "请先在工作空间根目录执行：\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  colcon build --packages-select course_interfaces nav2_mission_manager\n"
            "  source install/setup.bash\n"
            "或在已 source 的终端里再运行本脚本。"
        )
    return None


def run(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    print(f"\n>>> {' '.join(cmd)}")
    return subprocess.run(
        cmd,
        cwd=cwd or PKG_ROOT,
        env=env or os.environ.copy(),
        text=True,
        capture_output=True,
    )


def check_flake8(env: dict[str, str]) -> dict:
    proc = run(
        [
            PYTHON,
            "-m",
            "flake8",
            "nav2_mission_manager",
            "--count",
            "--statistics",
            "--max-line-length=100",
            "--extend-ignore=E501,W391,E302,E303",
        ],
        env=env,
    )
    print(proc.stdout)
    if proc.stderr:
        print(proc.stderr, file=sys.stderr)
    errors = 0
    for line in proc.stdout.splitlines():
        if line.strip().isdigit():
            errors = int(line.strip())
            break
    return {
        "tool": "flake8",
        "passed": proc.returncode == 0,
        "error_count": errors,
        "summary": "无严重风格/语法问题" if proc.returncode == 0 else f"flake8 报告 {errors} 项",
    }


def check_bandit(env: dict[str, str]) -> dict:
    proc = run([PYTHON, "-m", "bandit", "-r", "nav2_mission_manager", "-f", "json", "-ll"], env=env)
    if proc.returncode == 2 and "No module named bandit" in proc.stderr:
        return {
            "tool": "bandit",
            "passed": True,
            "skipped": True,
            "summary": "未安装 bandit，跳过安全扫描",
        }
    if proc.stdout:
        print(proc.stdout[:2000])
    try:
        data = json.loads(proc.stdout or "{}")
    except json.JSONDecodeError:
        data = {}
    issues = data.get("results", [])
    high_med = [i for i in issues if i.get("issue_severity") in {"HIGH", "MEDIUM"}]
    return {
        "tool": "bandit",
        "passed": len(high_med) == 0,
        "issue_count": len(issues),
        "high_medium_count": len(high_med),
        "summary": "无 high/medium 安全漏洞" if not high_med else f"发现 {len(high_med)} 个 high/medium 问题",
    }


def run_pytest(with_html: bool, env: dict[str, str]) -> dict:
    cmd = [
        PYTHON,
        "-m",
        "pytest",
        "test/",
        "-q",
        "--cov=nav2_mission_manager",
        "--cov-branch",
        "--cov-config=.coveragerc",
        "--cov-report=term-missing:skip-covered",
    ]
    if with_html:
        cmd.append("--cov-report=html:test_reports/coverage_html")
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    proc = run(cmd, env=env)
    print(proc.stdout)
    if proc.stderr:
        print(proc.stderr, file=sys.stderr)

    if proc.returncode != 0 and "ModuleNotFoundError" in (proc.stdout + proc.stderr):
        print(
            "\n[提示] pytest 收集失败，覆盖率会变成 0%。"
            "通常是未 colcon build 或未 source install/setup.bash。",
            file=sys.stderr,
        )

    json_proc = run([PYTHON, "-m", "coverage", "json", "-o", "test_reports/coverage.json"], env=env)
    if json_proc.stdout:
        print(json_proc.stdout)
    cov_path = REPORT_DIR / "coverage.json"
    stmt_pct = branch_pct = 0.0
    if cov_path.exists():
        cov = json.loads(cov_path.read_text(encoding="utf-8"))
        totals = cov.get("totals", {})
        stmt_pct = float(totals.get("percent_covered", 0.0))
        covered_br = totals.get("covered_branches", 0)
        num_br = totals.get("num_branches", 1) or 1
        branch_pct = 100.0 * covered_br / num_br
    passed_tests = proc.returncode == 0
    passed_cov = stmt_pct >= STMT_MIN and branch_pct >= BRANCH_MIN
    return {
        "tool": "pytest+coverage",
        "passed": passed_tests and passed_cov,
        "tests_passed": passed_tests,
        "statement_coverage_pct": round(stmt_pct, 2),
        "branch_coverage_pct": round(branch_pct, 2),
        "statement_threshold_pct": STMT_MIN,
        "branch_threshold_pct": BRANCH_MIN,
        "summary": (
            f"语句 {stmt_pct:.1f}% / 分支 {branch_pct:.1f}% "
            f"(阈值 {STMT_MIN:.0f}% / {BRANCH_MIN:.0f}%)"
        ),
    }


def write_report(results: list[dict]) -> Path:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "package": "nav2_mission_manager",
        "owner": "徐梓鸣",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "quality_gates": results,
        "overall_passed": all(r.get("passed", False) for r in results),
    }
    out = REPORT_DIR / "quality_gate_report.json"
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Run nav2_mission_manager quality gates")
    parser.add_argument("--html", action="store_true", help="Generate HTML coverage report")
    args = parser.parse_args()

    env = build_test_env()
    err = check_prerequisites(env)
    if err:
        print(f"环境检查失败:\n{err}", file=sys.stderr)
        return 1

    results = [check_flake8(env), check_bandit(env), run_pytest(args.html, env)]
    report_path = write_report(results)

    print("\n========== 质量门禁汇总 ==========")
    for item in results:
        status = "PASS" if item.get("passed") else "FAIL"
        print(f"[{status}] {item['tool']}: {item.get('summary', '')}")
    print(f"\n报告已写入: {report_path}")

    return 0 if all(r.get("passed") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
