from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gpspackage')
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # EKF Düğümü (Filtreleme)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file]
    )

    # NavSat Transform Düğümü (Enlem/Boylam -> Metre)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/local')
        ]
    )

    return LaunchDescription([
        ekf_node,
        navsat_node
    ])
