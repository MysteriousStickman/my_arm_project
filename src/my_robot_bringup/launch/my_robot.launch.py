from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context).lower()
    port = LaunchConfiguration("port").perform(context)
    speed_deg_s = LaunchConfiguration("speed_deg_s").perform(context)

    if use_fake_hardware not in ("true", "false"):
        raise ValueError("use_fake_hardware must be 'true' or 'false'")

    bringup_xacro = str(
        Path(get_package_share_directory("my_robot_bringup"))
        / "config"
        / "alicia_d.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "Alicia_D_v5_6_gripper_50mm", package_name="moveit_config"
        )
        .robot_description(
            file_path=bringup_xacro,
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "hw_port": port,
                "hw_gripper_type": "50mm",
                "hw_default_speed_deg_s": speed_deg_s,
            }
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution(
                [FindPackageShare("moveit_config"), "config", "ros2_controllers.yaml"]
            ),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    commander = Node(
        package="my_commander",
        executable="commander_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("moveit_config"), "config", "moveit.rviz"]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    start_arm_controller = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])
    )
    start_gripper_controller = RegisterEventHandler(
        OnProcessExit(target_action=arm_controller, on_exit=[gripper_controller])
    )
    start_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller,
            on_exit=[move_group, commander, rviz],
        )
    )

    mode = "FAKE" if use_fake_hardware == "true" else "REAL"
    return [
        LogInfo(msg=f"Starting Alicia-D in {mode} hardware mode"),
        world_to_base,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        start_arm_controller,
        start_gripper_controller,
        start_moveit,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use mock hardware. Set false only when the real robot is ready.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description="Serial port for real hardware; empty enables auto-detection.",
            ),
            DeclareLaunchArgument(
                "speed_deg_s",
                default_value="0.1",
                description="Real robot command speed in degrees per second.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
