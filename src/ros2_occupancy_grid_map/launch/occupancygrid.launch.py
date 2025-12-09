from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import GroupAction

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("occupancy_grid_map"),
        "config",
        "occupancy_grid_map.param.yaml",
    )
    # Declare any necessary launch arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )
    pointcloud_topic = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/cloudPointMapping",
    )
    target_frame = DeclareLaunchArgument(
        "target_frame",
        default_value="map",
    )
    occupancy_grid_map_node = Node(
        package="occupancy_grid_map",
        executable="occupancy_grid_map_exe",
        name="occupancy_grid_map_node",
        namespace="",
        parameters=[
            config,
            {
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        output="both",
    )
    # Create a group action to launch nodes in parallel
    parallel_group = GroupAction([use_sim_time, pointcloud_topic, target_frame, occupancy_grid_map_node])

    return LaunchDescription([
        parallel_group,
        LogInfo(msg="All nodes launched in parallel!"),
    ])
