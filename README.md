# Merged Nav2 Course Workspace

This repository is the unified ROS 2 Humble workspace for the four-module course project:

- `sam_bot_nav2_gz`: Gazebo + Nav2 base bringup
- `nav2_mission_manager`: mission action server and state machine
- `semantic_costmap_plugins`: semantic costmap layers
- `sam_bot_safety_monitor`: safety monitor and demo helpers
- `nav2_scenario_runner`: scenario generation and batch execution
- `course_bringup`: integrated launch entrypoints
- `course_interfaces`: shared messages, services, and actions

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Manual Demo

```
python3 src/sam_bot_nav2_gz/scripts/generate_scattered_room.py --seed 42 --box-count 12
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch course_bringup mission_demo.launch.py \
  world_file:=demo_pillar_room.sdf \
  enable_semantic:=True \
  enable_safety:=True
```



```bash
ros2 launch course_bringup mission_demo.launch.py run_headless:=False
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 action send_goal /mission/run course_interfaces/action/RunMission \
'{mission_file: "src/course_bringup/config/sample_mission.json", max_retry_per_waypoint: 1, allow_skip_waypoint: false}'
```

## Batch Scenario Demo

```bash
python3 src/nav2_scenario_runner/scripts/generate_cases.py \
  --profile src/nav2_scenario_runner/config/batch_profiles/default_batch.yaml \
  --output-dir /tmp/nav2_cases
  
python3 src/nav2_scenario_runner/scripts/run_batch.py \
  --cases-dir /tmp/nav2_cases \
  --results-dir /tmp/nav2_results_gui \
  --launch-package nav2_scenario_runner \
  --launch-file run_single_case.launch.py \
  --timeout-sec 300 \
  --extra-launch-arg run_headless:=False \
  --extra-launch-arg enable_safety:=True

ros2 run nav2_scenario_runner summarize_results.py --results-dir /tmp/nav2_results
```

## Docker

See `how_to_run` for the recommended Docker workflow.
