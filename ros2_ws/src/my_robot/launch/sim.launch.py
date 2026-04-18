import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("my_robot")

    # ── 参数 ────────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="使用仿真时钟")
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="是否显示 Gazebo GUI")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_share, "worlds", "my_world.world"),
        description="Gazebo 世界文件路径")
    x_arg = DeclareLaunchArgument("x_pose", default_value="0.0")
    y_arg = DeclareLaunchArgument("y_pose", default_value="0.0")
    z_arg = DeclareLaunchArgument("z_pose", default_value="0.10")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui          = LaunchConfiguration("gui")
    world        = LaunchConfiguration("world")
    x_pose       = LaunchConfiguration("x_pose")
    y_pose       = LaunchConfiguration("y_pose")
    z_pose       = LaunchConfiguration("z_pose")

    # ── URDF/xacro 处理 ──────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    robot_description_content = Command(["xacro ", xacro_file])

    robot_description = {
        "robot_description": robot_description_content,
        "use_sim_time": use_sim_time,
    }

    # ── robot_state_publisher ────────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── Gazebo 服务器 ────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"),
                         "launch", "gazebo.launch.py"),
        ]),
        launch_arguments={
            "world": world,
            "gui": gui,
        }.items(),
    )

    # ── 将机器人模型生成到 Gazebo ─────────────────────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_robot",
            "-x", x_pose, "-y", y_pose, "-z", z_pose,
        ],
        output="screen",
    )

    # ── 加载控制器（在机器人生成完毕后执行） ─────────────────────────────────
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "diff_drive_controller"],
        output="screen",
    )

    # 在 spawn 完成后加载控制器
    activate_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                load_joint_state_broadcaster,
                load_diff_drive_controller,
            ],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        world_arg,
        x_arg, y_arg, z_arg,

        rsp_node,
        gazebo_launch,
        spawn_robot,
        activate_controllers,
    ])
