"""pytest 公共 fixture：ROS 上下文与 mission JSON。"""
import json

import pytest
import rclpy


@pytest.fixture(scope="session")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def sample_mission_file(tmp_path):
    path = tmp_path / "mission.json"
    path.write_text(
        json.dumps(
            {
                "mission_id": "test_mission",
                "frame_id": "map",
                "waypoints": [
                    {"id": "W1", "x": 1.0, "y": 2.0, "yaw": 0.0},
                    {"id": "W2", "x": 3.0, "y": 4.0, "yaw": 1.57},
                ],
            }
        ),
        encoding="utf-8",
    )
    return path
