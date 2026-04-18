import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("my_robot")

    # ── 参数 ────────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz_config = os.path.join(pkg_share, "rviz", "robot.rviz")

    # ── rviz2 ────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ── rqt（通用 GUI，可开 Topic Monitor / TF Tree 等插件） ───────────────
    rqt_node = ExecuteProcess(
        cmd=["rqt"],
        output="screen",
    )

    # ── rqt_graph（节点拓扑图） ───────────────────────────────────────────
    rqt_graph_node = ExecuteProcess(
        cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
        rqt_node,
        # rqt_graph_node,   # 取消注释可同时弹出节点图
    ])
