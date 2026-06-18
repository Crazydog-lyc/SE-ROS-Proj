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
python3 src/sam_bot_nav2_gz/scripts/generate_scattered_room.py --seed 53 --box-count 7
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch course_bringup mission_demo.launch.py \
  world_file:=demo_pillar_room.sdf \
  enable_semantic:=True \
  enable_safety:=True
```

## Batch Scenario Demo

```bash
GENERATE_SCATTERED_WORLD=True SCATTER_SEED=53 SCATTER_BOX_COUNT=7 RUN_HEADLESS=False RESULTS_DIR=/tmp/nav2_results_gui 
bash src/nav2_scenario_runner/scripts/run_batch.sh
```

