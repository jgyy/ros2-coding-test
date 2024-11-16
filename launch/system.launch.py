from launch import LaunchDescription
from launch_ros.action import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='temperature_system')
    log_level = LaunchConfiguration('log_level', default='info')
    publish_rate = LaunchConfiguration('publish_rate', default='1.0')
    temp_min = LaunchConfiguration('temp_min', default='20.0')
    temp_max = LaunchConfiguration('temp_max', default='30.0')
    default_unit = LaunchConfiguration('default_unit', default='celsius')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='temperature_system',
            description='Namespace for all nodes'),
        DeclareLaunchArgument('log_level', default_value='info',
            description='Logging level'),
        DeclareLaunchArgument('publish_rate', default_value='1.0',
            description='Temperature publishing rate in Hz'),
        Node(package='basic_pubsub', namespace=namespace, executable='publisher',
            name='temp_publisher',
            parameters=[{'publish_rate': publish_rate, 'temp_min': temp_min,
                'temp_max': temp_max, 'default_unit': default_unit}],
            arguments=['--ros-args', '--log-level', log_level], output='screen'),
        Node(package='basic_pubsub', namespace=namespace, executable='subscriber',
            name='temp_subscriber',
            arguments=['--ros-args', '--log-level', log_level], output='screen'),
        Node(package='basic_pubsub', namespace=namespace, executable='temp_service',
            name='temp_conversion',
            arguments=['--ros-args', '--log-level', log_level], output='screen'),
        Node(package='robot_monitor', namespace=namespace,
            executable='battery_server', name='mattery_monitor',
            arguments=['--ros-args', '--log-level', log_level], output='screen'),
        LogInfo(msg=['Starting temperature monitoring system in namespace :',
            namespace])
    ])
