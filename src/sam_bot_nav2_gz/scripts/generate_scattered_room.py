# ========================================================================
# 文件: src/sam_bot_nav2_gz/scripts/generate_scattered_room.py
# 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
# ========================================================================
#
# 【AI-PROMPT】
# Python 脚本：程序化生成 Gazebo SDF 房间，围墙+中心柱+随机散布 box 障碍物，参数化 room_size/obstacle_count/seed，输出
# .sdf 文件。请生成 argparse 和 XML 模板框架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
import argparse
import math
import pathlib
import random
import textwrap

WALL_HALF_X = 3.3
WALL_HALF_Y = 2.5
WALL_MARGIN = 0.45
ROBOT_SPAWN = (-2.0, 0.0)
SPAWN_CLEAR_RADIUS = 0.85
PILLAR_CENTER = (0.0, 0.0)
PILLAR_RADIUS = 0.40
PILLAR_CLEAR_GAP = 0.25


def _boxes_overlap(
    ax: float, ay: float, aw: float, ah: float,
    bx: float, by: float, bw: float, bh: float,
    gap: float = 0.15,
) -> bool:
    return (
        abs(ax - bx) < (aw + bw) / 2.0 + gap
        and abs(ay - by) < (ah + bh) / 2.0 + gap
    )


def _in_spawn_clearance(x: float, y: float, w: float, h: float) -> bool:
    sx, sy = ROBOT_SPAWN
    return _boxes_overlap(
        x, y, w, h, sx, sy, 2 * SPAWN_CLEAR_RADIUS, 2 * SPAWN_CLEAR_RADIUS, gap=0.0
    )


def _in_pillar_clearance(x: float, y: float, w: float, h: float) -> bool:
    px, py = PILLAR_CENTER
    half_diag = math.hypot(w, h) / 2.0
    return math.hypot(x - px, y - py) < PILLAR_RADIUS + half_diag + PILLAR_CLEAR_GAP


# TODO[陆华均][FR-A] FR-A-06 程序化生成 pillar room 散布障碍物

def generate_boxes(seed: int, count: int) -> list[dict]:
    # TODO[陆华均]：FR-A-06 程序化生成 pillar room 散布障碍物，服务期末 Demo 场景
# TODO[陆华均][FR-A] FR-A-06 程序化生成 pillar room 散布障碍物，服务期末 Demo 场景
    rng = random.Random(seed)
    boxes: list[dict] = []
    max_attempts = count * 80

    for _ in range(max_attempts):
        if len(boxes) >= count:
            break

        width = rng.uniform(0.32, 0.72)
        depth = rng.uniform(0.28, 0.68)
        height = rng.uniform(0.42, 0.78)
        yaw = rng.uniform(-math.pi, math.pi)

        cos_y = abs(math.cos(yaw))
        sin_y = abs(math.sin(yaw))
        footprint_w = width * cos_y + depth * sin_y
        footprint_h = width * sin_y + depth * cos_y

        max_x = WALL_HALF_X - WALL_MARGIN - footprint_w / 2.0
        max_y = WALL_HALF_Y - WALL_MARGIN - footprint_h / 2.0
        if max_x <= 0 or max_y <= 0:
            continue

        x = rng.uniform(-max_x, max_x)
        y = rng.uniform(-max_y, max_y)

        if _in_spawn_clearance(x, y, footprint_w, footprint_h):
            continue
        if _in_pillar_clearance(x, y, footprint_w, footprint_h):
            continue

        overlaps = False
        for existing in boxes:
            if _boxes_overlap(
                x, y, footprint_w, footprint_h,
                existing["x"], existing["y"], existing["footprint_w"], existing["footprint_h"],
            ):
                overlaps = True
                break
        if overlaps:
            continue

        color = (
            rng.uniform(0.45, 0.88),
            rng.uniform(0.42, 0.78),
            rng.uniform(0.38, 0.72),
        )
        boxes.append(
            {
                "name": f"scatter_box_{len(boxes) + 1:02d}",
                "x": x,
                "y": y,
                "z": height / 2.0,
                "yaw": yaw,
                "width": width,
                "depth": depth,
                "height": height,
                "footprint_w": footprint_w,
                "footprint_h": footprint_h,
                "color": color,
            }
        )

    if len(boxes) < count:
        raise RuntimeError(f"Only placed {len(boxes)} boxes; try lowering count or margins.")

    return boxes


def _box_model_xml(box: dict) -> str:
    r, g, b = box["color"]
    return textwrap.dedent(
        f"""
    <model name="{box['name']}">
      <static>true</static>
      <pose>{box['x']:.3f} {box['y']:.3f} {box['z']:.3f} 0 0 {box['yaw']:.3f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{box['width']:.3f} {box['depth']:.3f} {box['height']:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{box['width']:.3f} {box['depth']:.3f} {box['height']:.3f}</size></box>
          </geometry>
          <material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient></material>
        </visual>
      </link>
    </model>
        """.strip()
    )


def build_world_sdf(seed: int, box_count: int) -> str:
    boxes = generate_boxes(seed, box_count)
    box_models = "\n\n".join(_box_model_xml(box) for box in boxes)

    return textwrap.dedent(
        f"""\
        <?xml version="1.0" ?>

        <sdf version="1.8">
          <world name="demo_pillar_room">
            <gui>
              <camera name="gzclient_camera">
                <pose>-0.5 0 12 0 1.45 0</pose>
              </camera>
            </gui>

            <physics name="1ms" type="ignored">
              <max_step_size>0.001</max_step_size>
              <real_time_factor>1</real_time_factor>
              <real_time_update_rate>1000</real_time_update_rate>
            </physics>

            <plugin name="ignition::gazebo::systems::Physics" filename="ignition-gazebo-physics-system"/>
            <plugin name="ignition::gazebo::systems::UserCommands" filename="ignition-gazebo-user-commands-system"/>
            <plugin name="ignition::gazebo::systems::SceneBroadcaster" filename="ignition-gazebo-scene-broadcaster-system"/>
            <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
              <render_engine>ogre2</render_engine>
            </plugin>

            <scene>
              <ambient>0.92 0.92 0.92 1</ambient>
              <background>0.86 0.89 0.93 1</background>
              <grid>false</grid>
            </scene>

            <light type="directional" name="sun">
              <cast_shadows>true</cast_shadows>
              <pose>0 0 10 0 0 0</pose>
              <diffuse>0.8 0.8 0.8 1</diffuse>
              <specular>0.2 0.2 0.2 1</specular>
              <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
              </attenuation>
              <direction>-0.4 0.2 -0.9</direction>
            </light>

            <model name="ground_plane">
              <static>true</static>
              <link name="link">
                <collision name="collision">
                  <geometry>
                    <plane>
                      <normal>0 0 1</normal>
                      <size>40 40</size>
                    </plane>
                  </geometry>
                </collision>
                <visual name="visual">
                  <geometry>
                    <plane>
                      <normal>0 0 1</normal>
                      <size>40 40</size>
                    </plane>
                  </geometry>
                  <material>
                    <ambient>0.96 0.94 0.9 1</ambient>
                    <diffuse>0.96 0.94 0.9 1</diffuse>
                    <specular>1 1 1 0</specular>
                  </material>
                </visual>
              </link>
            </model>

            <model name="sky_cam">
              <static>true</static>
              <link name="camera_link">
                <pose>0 0 12 1.57 1.57 0</pose>
                <sensor name="sky_cam" type="camera">
                  <camera>
                    <horizontal_fov>1.047</horizontal_fov>
                    <image>
                      <width>1024</width>
                      <height>768</height>
                    </image>
                    <clip>
                      <near>0.1</near>
                      <far>100</far>
                    </clip>
                  </camera>
                  <always_on>1</always_on>
                  <update_rate>1</update_rate>
                  <visualize>true</visualize>
                  <topic>sky_cam</topic>
                </sensor>
              </link>
            </model>

            <model name="north_wall">
              <static>true</static>
              <pose>0 {WALL_HALF_Y} 0.6 0 0 0</pose>
              <link name="link">
                <collision name="collision">
                  <geometry><box><size>7.0 0.2 1.2</size></box></geometry>
                </collision>
                <visual name="visual">
                  <geometry><box><size>7.0 0.2 1.2</size></box></geometry>
                  <material><ambient>0.75 0.78 0.83 1</ambient></material>
                </visual>
              </link>
            </model>

            <model name="south_wall">
              <static>true</static>
              <pose>0 -{WALL_HALF_Y} 0.6 0 0 0</pose>
              <link name="link">
                <collision name="collision">
                  <geometry><box><size>7.0 0.2 1.2</size></box></geometry>
                </collision>
                <visual name="visual">
                  <geometry><box><size>7.0 0.2 1.2</size></box></geometry>
                  <material><ambient>0.75 0.78 0.83 1</ambient></material>
                </visual>
              </link>
            </model>

            <model name="west_wall">
              <static>true</static>
              <pose>-{WALL_HALF_X} 0 0.6 0 0 0</pose>
              <link name="link">
                <collision name="collision">
                  <geometry><box><size>0.2 5.2 1.2</size></box></geometry>
                </collision>
                <visual name="visual">
                  <geometry><box><size>0.2 5.2 1.2</size></box></geometry>
                  <material><ambient>0.72 0.75 0.8 1</ambient></material>
                </visual>
              </link>
            </model>

            <model name="east_wall">
              <static>true</static>
              <pose>{WALL_HALF_X} 0 0.6 0 0 0</pose>
              <link name="link">
                <collision name="collision">
                  <geometry><box><size>0.2 5.2 1.2</size></box></geometry>
                </collision>
                <visual name="visual">
                  <geometry><box><size>0.2 5.2 1.2</size></box></geometry>
                  <material><ambient>0.72 0.75 0.8 1</ambient></material>
                </visual>
              </link>
            </model>

            <model name="main_pillar">
              <static>true</static>
              <pose>0.0 0.0 0.45 0 0 0</pose>
              <link name="link">
                <collision name="collision">
                  <geometry>
                    <cylinder><radius>0.40</radius><length>0.9</length></cylinder>
                  </geometry>
                </collision>
                <visual name="visual">
                  <geometry>
                    <cylinder><radius>0.40</radius><length>0.9</length></cylinder>
                  </geometry>
                  <material><ambient>0.78 0.56 0.42 1</ambient></material>
                </visual>
              </link>
            </model>

        {box_models}

          </world>
        </sdf>
        """
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate demo_pillar_room.sdf with random boxes")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--box-count", type=int, default=12)
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=None,
        help="Output SDF path (default: world/demo_pillar_room.sdf)",
    )
    args = parser.parse_args()

    script_dir = pathlib.Path(__file__).resolve().parent
    default_output = script_dir.parent / "world" / "demo_pillar_room.sdf"
    output_path = args.output or default_output

    sdf = build_world_sdf(args.seed, args.box_count)
    output_path.write_text(sdf, encoding="utf-8")
    print(f"Wrote {output_path} with seed={args.seed}, boxes={args.box_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
