# semantic_costmap_plugins (full fixed)

This is the full package version.

What is kept:
- SemanticZoneLayer
- PreferredLaneLayer
- DynamicCongestionLayer
- helper headers / cpp files
- demo scripts

What is changed:
- launch now wraps the original working `sam_bot_nav2_gz/complete_navigation.launch.py`
- planner is reverted to the original-style NavFn planner
- by default only `SemanticZoneLayer` is enabled
- the default semantic zone is placed far away at `(10.0, 10.0)` so it will not block the stock waypoint demo
- PreferredLaneLayer and DynamicCongestionLayer stay in the codebase and compile, but are disabled by default

Build:
```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_costmap_plugins
source install/setup.bash
```

Run:
```bash
ros2 launch semantic_costmap_plugins semantic_navigation.launch.py
```
