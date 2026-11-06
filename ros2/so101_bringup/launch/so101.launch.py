"""Bring up the SO-101 with ros2_control.

hardware:=real      the arm on a serial port (needs port:= and calibration_file:=)
hardware:=fake_bus  the real driver talking to a simulated servo bus on a PTY
hardware:=mock      ros2_control mock_components (no driver, no serial)

Examples:
  ros2 launch so101_bringup so101.launch.py hardware:=real port:=/dev/ttyACM0 \
      calibration_file:=~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_arm.json
  ros2 launch so101_bringup so101.launch.py hardware:=fake_bus rviz:=true
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro

FAKE_BUS_LINK = '/tmp/so101_fake_bus'


def _setup(context):
    hardware = LaunchConfiguration('hardware').perform(context)
    port = LaunchConfiguration('port').perform(context)
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    if hardware not in ('real', 'fake_bus', 'mock'):
        raise RuntimeError(f"hardware must be real, fake_bus or mock, got '{hardware}'")

    hw_share = FindPackageShare('so101_hardware').perform(context)
    desc_share = FindPackageShare('so101_description').perform(context)
    bringup_share = FindPackageShare('so101_bringup').perform(context)

    actions = []
    if hardware == 'fake_bus':
        port = FAKE_BUS_LINK
        if not calibration_file:
            calibration_file = os.path.join(hw_share, 'config', 'example_calibration.json')
        actions.append(ExecuteProcess(
            cmd=[os.path.join(get_package_prefix('so101_hardware'), 'lib', 'so101_hardware',
                              'fake_servo_bus'),
                 '--calibration', calibration_file, '--link', FAKE_BUS_LINK],
            name='fake_servo_bus', output='screen'))
    elif hardware == 'real' and not calibration_file:
        raise RuntimeError(
            'hardware:=real needs calibration_file:=<LeRobot calibration JSON>, e.g. '
            '~/.cache/huggingface/lerobot/calibration/robots/so_follower/<robot id>.json')

    xacro_file = os.path.join(desc_share, 'urdf', 'so101.urdf.xacro')
    robot_description = ParameterValue(_xacro(xacro_file, {
        'use_mock_hardware': 'true' if hardware == 'mock' else 'false',
        'port': port,
        'calibration_file': calibration_file,
    }), value_type=str)

    controllers = os.path.join(bringup_share, 'config', 'controllers.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    actions.append(rsp)
    if hardware == 'fake_bus':
        # Give the simulator a moment to create its PTY before the driver opens it.
        actions.append(TimerAction(period=1.0, actions=[control_node, spawner]))
    else:
        actions += [control_node, spawner]

    if use_rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(desc_share, 'rviz', 'so101.rviz')]))
    return actions


def _xacro(path, mappings):
    return xacro.process_file(path, mappings=mappings).toxml()


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock',
                              description='real | fake_bus | mock'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial device of the follower arm (hardware:=real)'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='LeRobot calibration JSON for this arm'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
