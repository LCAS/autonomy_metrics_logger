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


    return LaunchDescription([
        DeclareLaunchArgument(
            'gps_topic', default_value=gps_topic, description=''),
        DeclareLaunchArgument(
            'gps_odom_topic', default_value=gps_odom_topic, description=''),
        DeclareLaunchArgument(
            'battery_status', default_value=battery_status, description=''),
        DeclareLaunchArgument(
            'estop_status', default_value=estop_status, description=''),
        Node(
            package='autonomy_metrics',
            executable='metric_logger',
            output='screen',
            parameters=[{
                'gps_topic': '/gps_base/fix',
                'gps_odom_topic': '/gps_base/odometry',
                'battery_status': '/battery_status',
                'estop_status': '/estop_status'
            }],
            # env={
            #     'ROBOT_NAME': 'your_robot_name',
            #     'FARM_NAME': 'your_farm_name',
            #     'FIELD_NAME': 'your_field_name',
            #     'APPLICATION': 'your_application',
            #     'SCENARIO_NAME': 'your_scenario_name'
            # }
        )
    ])
