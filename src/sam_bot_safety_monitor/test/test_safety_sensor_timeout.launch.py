# ========================================================================
# 文件: src/sam_bot_safety_monitor/test/test_safety_sensor_timeout.launch.py
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# 请为这个模块生成 gtest/launch_test 骨架：一个 smoke 用例断言核心函数不抛异常，具体断言我后面补。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
import time
import unittest

import launch_testing.actions
import launch_testing.markers
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rclpy.node import Node as RclpyNode

from course_interfaces.msg import SafetyState
from course_interfaces.srv import GetSafetyState


@pytest.mark.launch_test
@launch_testing.markers.keep_alive

# 单测：def generate_description
def generate_test_description():
    safety_monitor = Node(
        package="sam_bot_safety_monitor",
        executable="safety_monitor_node.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "enable_tf_monitor": False,
                "enable_blockage_monitor": False,
                "startup_grace_period_sec": 0.1,
                "sensor_timeout_sec": 0.4,
                "tf_timeout_sec": 0.0,
                "blockage_window_sec": 0.0,
            }
        ],
    )

    return LaunchDescription(
        [
            safety_monitor,
            ReadyToTest(),
        ]
    ), {"safety_monitor": safety_monitor}


class TestSafetySensorTimeout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("test_safety_sensor_timeout_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _get_state(self):
        client = self.node.create_client(GetSafetyState, "/safety/get_safety_state")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(GetSafetyState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result())
        return future.result().state

    # 单测：def sensor_timeout_enters_paused
    def test_sensor_timeout_enters_paused(self):
        deadline = time.time() + 5.0
        state = self._get_state()
        while time.time() < deadline and state.state_label != "PAUSED":
            time.sleep(0.2)
            state = self._get_state()

        self.assertEqual(state.level, SafetyState.STOP_NOW)
        self.assertEqual(state.state_label, "PAUSED")
        self.assertEqual(state.source, "scan")
