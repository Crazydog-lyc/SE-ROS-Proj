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
from course_interfaces.srv import (
    GetSafetyState,
    TriggerEmergencyStop,
    TriggerPause,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
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
                "startup_grace_period_sec": 30.0,
                "sensor_timeout_sec": 60.0,
                "tf_timeout_sec": 60.0,
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


class TestSafetyServices(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode("test_safety_services_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, service_type, service_name, request):
        client = self.node.create_client(service_type, service_name)
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result())
        return future.result()

    def test_initial_state_is_normal(self):
        response = self._call_service(
            GetSafetyState,
            "/safety/get_safety_state",
            GetSafetyState.Request(),
        )
        self.assertEqual(response.state.level, SafetyState.SAFE)
        self.assertEqual(response.state.state_label, "NORMAL")

    def test_manual_pause_and_estop(self):
        pause_request = TriggerPause.Request()
        pause_request.reason = "test pause"
        pause_response = self._call_service(
            TriggerPause,
            "/safety/trigger_pause",
            pause_request,
        )
        self.assertTrue(pause_response.success)
        self.assertEqual(pause_response.state.level, SafetyState.STOP_NOW)
        self.assertEqual(pause_response.state.state_label, "PAUSED")

        estop_request = TriggerEmergencyStop.Request()
        estop_request.reason = "test estop"
        estop_response = self._call_service(
            TriggerEmergencyStop,
            "/safety/trigger_emergency_stop",
            estop_request,
        )
        self.assertTrue(estop_response.success)
        self.assertEqual(estop_response.state.level, SafetyState.STOP_NOW)
        self.assertEqual(estop_response.state.state_label, "EMERGENCY_STOP")
