# nav2_scenario_runner

Plugin-based scenario generation and batch evaluation framework for Nav2.

## Provided plugins

- nav2_scenario_runner::plugins::CorridorGenerator
- nav2_scenario_runner::plugins::RoomInspectionGenerator
- nav2_scenario_runner::plugins::CongestionGenerator
- nav2_scenario_runner::plugins::FaultInjectionGenerator

## Build

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_scenario_runner
source install/setup.bash
```

## Generate cases

```bash
ros2 run nav2_scenario_runner generate_cases.py   --profile src/nav2_scenario_runner/config/batch_profiles/default_batch.yaml   --output-dir /tmp/nav2_cases
```

## Run a batch

```bash
ros2 run nav2_scenario_runner run_batch.py   --cases-dir /tmp/nav2_cases   --results-dir /tmp/nav2_results   --launch-package nav2_scenario_runner   --launch-file run_single_case.launch.py   --timeout-sec 60
```

## Summarize results

```bash
ros2 run nav2_scenario_runner summarize_results.py   --results-dir /tmp/nav2_results
```

Notes:
- `run_single_case.launch.py` now forwards to `course_bringup/scenario_case.launch.py`.
- Generated case directories also include `<case_id>_mission.json` and `<case_id>_semantic_overlay.yaml` companion files for the merged workspace.
