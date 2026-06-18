# SBOM 清单与代码说明文档

**项目名称**：SE-ROS-Proj  
**日期**：2026-06-18  
**版本**：2.0

---

## 1. 文档说明

本文档按 **文件组织结构** 说明系统构成，并对每个模块标注 **来源分类**：

| 代码 | 含义 |
|------|------|
| **O** | 复用开源（Open Source）— 官方/第三方仓库直接沿用或少量修改 |
| **L** | 大模型自动生成（LLM）— 【AI-SCOPE】骨架，提示词见 §2 |
| **G** | 其他方法生成 — 脚本/工具程序化生成（非 LLM 业务逻辑） |
| **S** | 自研（Self-developed）— `TODO[姓名]` 标记的业务实现 |

**注释约定**（自研部分）：

- 文件头：`负责人` / `需求: FR-X` / `【AI-PROMPT】` / `【AI-SCOPE】`
- 业务逻辑：`# TODO[姓名]：FR-x-yy 说明` 或 `// TODO[姓名]：...`
- 联调：`# LINK[姓名]：...`

规范全文：`report/成员分工_提示词与注释规范.md`

---

## 2. 大模型提示词汇总与人工加工说明

### 2.1 通用提示词模板

```
我在 ROS2 Humble + Nav2 课程项目里，需要 [模块名]。
请搭 [类/节点/launch] 骨架：[接口列表]；
[明确不要生成的部分，如：状态机/融合逻辑我自己写]。
请生成 import · declare · register · 空壳类/函数即可。
```

### 2.2 各模块典型提示词与人工加工

| 模块 | 典型【AI-PROMPT】（摘自源码头） | AI 生成范围（【AI-SCOPE】） | 人工加工（S） |
|------|--------------------------------|----------------------------|---------------|
| **FR-B 任务** | MissionActionServerNode：ActionServer 执行循环里 poll safety、goal timeout、调用 state_machine | import、declare、Action Server 注册、回调空壳 | `state_machine.py` 全部 `_handle_*` 转移逻辑；retry；mission_loader 校验；navigator_adapter lifecycle 轮询；安全备份链 |
| **FR-B 接口** | RunMission.action / MissionState.msg 字段骨架 | msg/action 字段定义 | 字段语义注释、与 mission 状态字符串对齐 |
| **FR-A 场景** | scaffold nav2_scenario_runner：pluginlib ScenarioGenerator、registry、serializer | 包结构、类声明、插件宏、空 `generate()` | 各 Generator 墙/waypoint/semantic 算法；`generate_cases.py` overlay 合并；`run_batch.py` 清理/tee |
| **FR-C 语义** | 基于 CostmapLayer 新建 SemanticZone/PreferredLane/DynamicCongestion 三插件骨架 | onInitialize/updateBounds/updateCosts 空壳、pluginlib | geometry_utils 射线法/投影；cost_functions 融合；zone 遍历与 task_mode 过滤 |
| **FR-D 安全** | SafetyMonitor：`_monitor_tick` 和各 `_check_*` 方法声明框架 | 参数 declare、订阅/服务注册 | 堵塞检测逻辑、spin 忽略、cancel Nav2、recovery 窗口、SafetyAwareNavigator |
| **集成 launch** | full_stack.launch.py：OpaqueFunction + Include complete_navigation | DeclareLaunchArgument、Include 骨架 | spawn 参数传递、mission_manager 参数文件、python3.10 路径 |
| **World/SDF** | 程序化 pillar room：argparse + XML 模板框架 | argparse、SDF 外层结构 | 碰撞检测 `_boxes_overlap`、spawn 净空、随机 box 布局 |
| **Batch 脚本** | generate_cases.py：argparse + profile 解析 + subprocess 框架 | CLI 骨架 | mission 过滤近 spawn 点、semantic overlay yaml 注入 |

### 2.3 迭代方式（课程要求）

1. **第一轮**：Cursor/LLM 生成骨架（【AI-SCOPE】）  
2. **第二轮**：补 pytest/gtest 框架  
3. **第三轮**：人工填充 TODO（S），联调修复（spin 180°、batch 清理、安全备份链）

---

## 3. 第三方开源组件 SBOM（运行时依赖）

| 组件 | 版本/发行版 | 用途 | 许可证 |
|------|-------------|------|--------|
| ROS 2 | Humble | 中间件 | Apache 2.0 |
| Nav2 | Humble deb | 导航栈 | Apache 2.0 |
| nav2_simple_commander | Humble | Python 导航封装 | Apache 2.0 |
| SLAM Toolbox | Humble | 在线 SLAM | LGPL / 项目许可 |
| Ignition Gazebo | 6.x | 仿真 | Apache 2.0 |
| ros_ign_gazebo / ros_ign_bridge | Humble | GZ↔ROS | Apache 2.0 |
| ros2_control / diff_drive_controller | Humble | 差分驱动 | Apache 2.0 |
| robot_localization | Humble | EKF | BSD |
| pluginlib | Humble | 插件加载 | BSD |
| yaml-cpp | system | YAML 解析 | MIT |
| AWS RoboMaker Retail Models | bundled | Gazebo 模型 | AWS 示例许可 |
| pytest / launch_testing | dev | 测试 | MIT |

**基线包 `sam_bot_nav2_gz`**：源自 Nav2 Gazebo 官方教程示例，标记为 **O**，不计四人自研工作量。

---

## 4. 仓库文件树与来源说明

### 4.1 根目录

| 路径 | 来源 | 用途 |
|------|------|------|
| `README.md` | S | 构建与 Demo 入口说明 |
| `docs/` | S/G | **本套提交制品** |
| `report/` | S | 分工规范、PPT 素材 |
| `src/` | — | 全部 ROS 包源码 |
| `build/`, `install/`, `log/` | G | colcon 产物（不提交） |

---

### 4.2 `course_interfaces/` — 共享接口 **O+L**

| 路径 | 来源 | 用途 |
|------|------|------|
| `action/RunMission.action` | L+S | 任务 Action 定义 |
| `msg/MissionState.msg` | L+S | `/mission/state` |
| `msg/SafetyState.msg` | L+S | `/safety/state` |
| `msg/ZoneRule.msg`, `ZoneRuleArray.msg` | L+S | 语义区域规则 |
| `msg/BatchTestResult.msg` | L+S | 批量测试结果 |
| `srv/GetSafetyState.srv` 等 | L+S | 安全服务 |
| `srv/ReloadZones.srv` | L+S | 语义热重载 |
| `srv/LoadScenario.srv` | L+S | 场景加载接口 |

---

### 4.3 `sam_bot_nav2_gz/` — 仿真基线 **O + L + S**

| 路径 | 来源 | 用途 |
|------|------|------|
| `launch/display.launch.py` | O+S | Gazebo、bridge、ros2_control、spawn |
| `launch/complete_navigation.launch.py` | O+S | SLAM → 延迟 → Nav2；BT 参数注入 |
| `launch/nav2_params_utils.py` | S | 运行时 patch nav2 params 路径 |
| `config/nav2_params.yaml` | O+S | Nav2 基线；behavior_server/recovery 调参 |
| `config/behavior_trees/*.xml` | O+S | 自定义 recovery：backup 0.25m → spin 180° |
| `config/safety_monitor.yaml` | L+S | safety 参数 |
| `scripts/generate_scattered_room.py` | L+G+S | 程序化 SDF 生成（FR-A-06） |
| `world/demo_pillar_room.sdf` | G | `generate_scattered_room.py` 生成 |
| `test/test_*.launch.py` | L+S | 启动集成测试 |

---

### 4.4 `nav2_mission_manager/` — 任务管理 **L + S**（徐梓鸣）

| 路径 | 来源 | 用途 |
|------|------|------|
| `nav2_mission_manager/mission_action_server.py` | L+S | `/mission/run` Action Server 主节点 |
| `nav2_mission_manager/state_machine.py` | L+S | 显式状态机 **（核心自研）** |
| `nav2_mission_manager/navigator_adapter.py` | L+S | BasicNavigator 封装 |
| `nav2_mission_manager/retry_policy.py` | S | 重试/跳过策略 |
| `scripts/run_quality_gate.py/sh` | S | flake8+bandit+pytest 门禁 |
| `test/test_*.py` | L+S | 78 个 pytest 用例，语句覆盖率 **92.8%** |

---

### 4.5 `nav2_scenario_runner/` — 场景与 Batch **L + S**（陆华均）

| 路径 | 来源 | 用途 |
|------|------|------|
| `src/plugins/*.cpp` | L+S | 四个 Generator **（核心自研）** |
| `scripts/generate_cases.py` | L+S | batch profile → mission/overlay |
| `scripts/run_batch.py` | L+S | 顺序 launch、log tee、进程清理 |
| `scripts/run_batch.sh` | S | 一键 batch |
| `config/batch_profiles/default_batch.yaml` | S | README 默认 2 case |
| `config/batch_profiles/full_batch.yaml` | S | 四场景全量 batch |
| `test/test_*.cpp` | L+S | gtest |

---

### 4.6 `semantic_costmap_plugins/` — 语义 Costmap **L + S**（李熠城）

| 路径 | 来源 | 用途 |
|------|------|------|
| `src/semantic_zone_layer.cpp` | L+S | 禁行/软代价 **（核心自研）** |
| `src/dynamic_congestion_layer.cpp` | L+S | 动态拥堵圆 |
| `src/geometry_utils.cpp` | L+S | 几何算法 **（核心自研）** |
| `src/cost_functions.cpp` | L+S | max/add/overwrite 融合 |
| `config/nav2_params_semantic.yaml` | L+S | Nav2 + 插件参数 |
| `test/test_geometry_utils.cpp` | L+S | gtest |
| `test/test_cost_functions.cpp` | L+S | gtest |

---

### 4.7 `sam_bot_safety_monitor/` — 安全监控 **L + S**（苏易）

| 路径 | 来源 | 用途 |
|------|------|------|
| `sam_bot_safety_monitor/safety_monitor.py` | L+S | 主监控节点 **（核心自研）** |
| `sam_bot_safety_monitor/safety_navigation.py` | L+S | SafetyAwareNavigator |
| `test/test_safety_*.launch.py` | L+S | launch 测试 |

---

### 4.8 `course_bringup/` — 集成 **L + S**

| 路径 | 来源 | 用途 |
|------|------|------|
| `launch/full_stack.launch.py` | L+S | Gazebo+Nav2+mission+safety 集成 |
| `launch/scenario_case.launch.py` | L+S | 单 case + auto_run + Shutdown |
| `scripts/auto_run_mission.py` | L+S | 等 Nav2 → 发 `/mission/run` |

---

## 5. 源码规模与来源占比（估算）

| 包 | 主要语言 | 自研(S) 占比（估） | 说明 |
|----|----------|-------------------|------|
| nav2_mission_manager | Python | ~70% | 状态机+测试为 S |
| nav2_scenario_runner | C++/Python | ~65% | Generator 与 batch 为 S |
| semantic_costmap_plugins | C++ | ~60% | 几何与 updateCosts 为 S |
| sam_bot_safety_monitor | Python | ~65% | 检测逻辑为 S |
| course_bringup | Python | ~50% | launch 集成为 S |
| sam_bot_nav2_gz | py/yaml/sdf | ~20% | 基线 O 为主 |
| course_interfaces | msg/srv | ~30% | 定义少，多 L 骨架 |

---

## 6. 构建产物 SBOM（install 空间）

`colcon build` 后主要可执行/资源：

| 安装路径 | 来源包 | 用途 |
|----------|--------|------|
| `lib/nav2_mission_manager/mission_manager_node` | mission_manager | 任务节点 |
| `lib/nav2_scenario_runner/generate_scenario_node` | scenario_runner | 场景生成 CLI |
| `lib/course_bringup/auto_run_mission.py` | course_bringup | 自动发 mission |
| `share/sam_bot_nav2_gz/config/*` | sam_bot | Nav2/safety/BT 参数 |
| `share/semantic_costmap_plugins/*` | semantic | 插件 .so + yaml |
| `share/nav2_scenario_runner/scripts/*` | scenario | batch 脚本 |

---

## 7. 许可证与合规

- 自研模块：课程项目授权，四人按分工维护  
- Nav2/Gazebo/AWS 模型：遵循各自上游许可证，**不得移除**原有版权声明  
- LLM 生成骨架：经人工审查、测试后合入，业务逻辑以 `TODO[姓名]` 自研部分为准  

---

## 8. 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-06-17 | 初版提交制品 |
| 2.0 | 2026-06-18 | 更新覆盖率 92.8%、Recovery 180° spin、batch profile 说明 |
