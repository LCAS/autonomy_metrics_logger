import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomy_metrics')

    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value= os.path.join(pkg_dir, 'config', 'metrics_full.yaml'),
        description='Path to YAML configuration file for mdbi_logger'
    )

    node = Node(
        package='autonomy_metrics',
        executable='metric_logger',  # ensure your setup/install maps executable correctly
        name='mdbi_logger',
        output='screen',
        parameters=[{
            'config_yaml': LaunchConfiguration('config_yaml'),
            # optional override
            'mongodb_host': 'localhost',
            'mongodb_port': 27018,
        }]
    )

    return LaunchDescription([config_yaml_arg, node])
