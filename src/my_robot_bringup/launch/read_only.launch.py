"""Show measured SDK feedback without activating robot controllers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    pkg = Path(get_package_share_directory("alicia_d_descriptions"))
    gripper_type = LaunchConfiguration("gripper_type").perform(context)
    urdf = pkg / "urdf/Alicia_D_v5_6" / f"Alicia_D_v5_6_gripper_{gripper_type}.urdf"
    description = {"robot_description": ParameterValue(urdf.read_text(), value_type=str)}
    reader = Node(
        package="my_robot_bringup", executable="alicia_read_only_state.py",
        output="screen",
        parameters=[{
            key: ParameterValue(LaunchConfiguration(key), value_type=str)
            for key in ["sdk_python", "sdk_path", "port", "expected_serial", "gripper_type"]
        }],
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=reader,
            on_exit=[EmitEvent(event=Shutdown(reason="Read-only SDK reader stopped"))],
        )),
        reader,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description], output="screen"),
        Node(package="rviz2", executable="rviz2", parameters=[description],
             arguments=["-d", str(pkg / "launch/alicia.rviz")],
             condition=IfCondition(LaunchConfiguration("use_rviz")), output="screen"),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("my_robot_bringup"))
    sdk_path = next(
        (parent / "Alicia-D-SDK" for parent in share.parents
         if (parent / "Alicia-D-SDK/alicia_d_sdk").is_dir()),
        Path.home() / "workspace/learn_ros_moveit/Alicia-D-SDK",
    )
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("expected_serial", default_value=""),
        DeclareLaunchArgument("sdk_path", default_value=str(sdk_path)),
        DeclareLaunchArgument("sdk_python", default_value=str(
            Path.home() / "miniconda3/envs/alicia_d_sdk/bin/python")),
        DeclareLaunchArgument("gripper_type", default_value="50mm", choices=["50mm", "100mm"]),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
