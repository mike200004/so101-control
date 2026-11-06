"""End-to-end test: full bringup against the simulated servo bus.

Exercises everything the real arm uses except the servos themselves:
xacro -> controller_manager -> so101_hardware plugin -> POSIX serial port
(a pseudo-terminal) -> Feetech protocol -> simulated STS3215s, and back.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ALL_JOINTS = ARM_JOINTS + ['gripper']


@pytest.mark.launch_test
def generate_test_description():
    bringup = os.path.join(
        get_package_share_directory('so101_bringup'), 'launch', 'so101.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'hardware': 'fake_bus'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFakeBusBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('so101_bringup_test')
        cls.latest = None
        cls.node.create_subscription(JointState, '/joint_states', cls._on_joint_state, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        cls.latest = dict(zip(msg.name, msg.position))

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_1_joint_states_published(self):
        ok = self.spin_until(
            lambda: self.latest is not None and set(ALL_JOINTS) <= set(self.latest), 60.0)
        self.assertTrue(ok, f'no complete /joint_states; last={self.latest}')
        # The simulator starts every servo in the middle of its range = URDF zero.
        for j in ARM_JOINTS:
            self.assertAlmostEqual(self.latest[j], 0.0, delta=0.01, msg=j)

    def test_2_trajectory_reaches_goal(self):
        client = ActionClient(
            self.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.assertTrue(client.wait_for_server(timeout_sec=60.0), 'arm_controller not available')

        target = [0.3, -0.4, 0.5, 0.2, -0.6]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint(positions=target)
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]

        send = client.send_goal_async(goal)
        self.assertTrue(self.spin_until(send.done, 10.0))
        handle = send.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.assertTrue(self.spin_until(result.done, 15.0))
        self.assertEqual(
            result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

        self.spin_until(lambda: False, 0.5)
        for j, q in zip(ARM_JOINTS, target):
            # One encoder step is 1.5 mrad.
            self.assertAlmostEqual(self.latest[j], q, delta=0.01, msg=j)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15, 130, 143])
