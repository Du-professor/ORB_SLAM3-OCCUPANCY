from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('orbslam3_dense_ros2')
    occupancy_share = get_package_share_directory('occupancy_grid_map')

    # Launch arguments for flexibility
    use_sim_time = LaunchConfiguration('use_sim_time')
    vocabulary_file = LaunchConfiguration('vocabulary_file')
    settings_file = LaunchConfiguration('settings_file')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    enable_pangolin = LaunchConfiguration('enable_pangolin')
    start_point_to_scan = LaunchConfiguration('start_point_to_scan')
    pointcloud_in = LaunchConfiguration('pointcloud_in')
    start_slam_toolbox = LaunchConfiguration('start_slam_toolbox')
    start_occupancy = LaunchConfiguration('start_occupancy')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    occupancy_target_frame = LaunchConfiguration('occupancy_target_frame')
    start_rviz = LaunchConfiguration('start_rviz')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time for all nodes.'),
        DeclareLaunchArgument(
            'vocabulary_file',
            default_value=os.path.join(pkg_share, 'orb_slam3', 'Vocabulary', 'ORBvoc.txt.bin'),
            description='Path to ORB vocabulary file.'),
        DeclareLaunchArgument(
            'settings_file',
            default_value=os.path.join(pkg_share, 'orb_slam3', 'config', 'RGB-D', 'RealSense_D435i.yaml'),
            description='Path to ORB-SLAM3 camera settings.'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
    DeclareLaunchArgument('enable_pangolin', default_value='True'),
        DeclareLaunchArgument('start_point_to_scan', default_value='False'),
        DeclareLaunchArgument('pointcloud_in', default_value='/camera/depth/points'),
        DeclareLaunchArgument('start_slam_toolbox', default_value='False'),
        DeclareLaunchArgument('start_occupancy', default_value='True'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/cloudPointMapping'),
        DeclareLaunchArgument('occupancy_target_frame', default_value='map'),
        DeclareLaunchArgument('start_rviz', default_value='True'),
    ]

    orb_slam3_node = Node(
        package='orbslam3_dense_ros2',
        executable='orb_slam3_main',
        name='orb_slam3_ros2_node',
        output='screen',
        parameters=[{
            'vocabulary_file': vocabulary_file,
            'settings_file': settings_file,
            'enable_pangolin': enable_pangolin,
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'use_sim_time': use_sim_time,
        }]
    )

    pointcloud_to_laserscan_node = Node(
        package='orbslam3_dense_ros2',
        executable='point_to_scan.py',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'cloud_in': pointcloud_in,
            'scan_out': '/scan',
            'min_height': 0.1,
            'max_height': 1.0,
            'range_min': 0.3,
            'range_max': 5.0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(start_point_to_scan)
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/scan', '/scan'), ('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(start_slam_toolbox)
    )

    occupancy_node = Node(
        package='occupancy_grid_map',
        executable='occupancy_grid_map_exe',
        name='occupancy_grid_map_node',
        output='screen',
        parameters=[
            os.path.join(occupancy_share, 'config', 'occupancy_grid_map.param.yaml'),
            {
                'pointcloud_topic': pointcloud_topic,
                'target_frame': occupancy_target_frame,
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(start_occupancy)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'rviz_config.rviz')],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription(
        declare_args
        + [
            orb_slam3_node,
            pointcloud_to_laserscan_node,
            slam_toolbox_node,
            occupancy_node,
            rviz_node,
        ]
    )
