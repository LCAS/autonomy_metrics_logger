# -*- coding: utf-8 -*-
#! /usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    gps_topic = LaunchConfiguration('gps_topic', default='/gps_base/fix')
    gps_odom_topic = LaunchConfiguration('gps_odom_topic', default='/gps_base/odometry')
    battery_status = LaunchConfiguration('battery_status', default='/battery_status')
    estop_status = LaunchConfiguration('estop_status', default='/estop_status')

    # New parameters for MongoDB host and port
    mongodb_host = LaunchConfiguration('mongodb_host', default='localhost')
    mongodb_port = LaunchConfiguration('mongodb_port', default='27017')

    return LaunchDescription([
        # Declare Launch Arguments for the existing parameters
        DeclareLaunchArgument(
            'gps_topic', default_value=gps_topic, description='GPS topic for base fix'),
        DeclareLaunchArgument(
            'gps_odom_topic', default_value=gps_odom_topic, description='GPS topic for odometry'),
        DeclareLaunchArgument(
            'battery_status', default_value=battery_status, description='Battery status topic'),
        DeclareLaunchArgument(
            'estop_status', default_value=estop_status, description='E-stop status topic'),

        # Declare Launch Arguments for the new MongoDB parameters
        DeclareLaunchArgument(
            'mongodb_host', default_value=mongodb_host, description='MongoDB host address'),
        DeclareLaunchArgument(
            'mongodb_port', default_value=mongodb_port, description='MongoDB port'),

        # Node Configuration
        Node(
            package='autonomy_metrics',
            executable='metric_logger',
            output='screen',
            parameters=[{
                'gps_topic': gps_topic,
                'gps_odom_topic': gps_odom_topic,
                'battery_status': battery_status,
                'estop_status': estop_status,
                'mongodb_host': mongodb_host,
                'mongodb_port': mongodb_port,
            }],
        )
    ])
