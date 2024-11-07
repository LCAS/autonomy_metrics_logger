# -*- coding: utf-8 -*-
#! /usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    gps_topic = LaunchConfiguration('gps_topic', default='/gps_base/fix')
    odometry_topic = LaunchConfiguration('odometry_topic', default='/diff_drive_controller/odom')
    battery_status = LaunchConfiguration('battery_status_topic', default='/diff_drive_controller/battery_status')
    estop_status = LaunchConfiguration('estop_status_topic', default='/diff_drive_controller/estop_status')
    hunter_status = LaunchConfiguration('hunter_status_topic', default='/hunter_status')
    actioned_by_coordinator_topic = LaunchConfiguration('actioned_by_coordinator_topic', default='/topological_navigation/execute_policy_mode/goal')
    
    min_distance_threshold = LaunchConfiguration('min_distance_threshold', default='0.2')

    # Path to the scenario and navigation repos
    aoc_scenario_path = LaunchConfiguration('aoc_scenario_path', default='/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario')
    aoc_navigation_path = LaunchConfiguration('aoc_navigation_path', default='/home/ros/aoc_strawberry_scenario_ws/src/aoc_navigation')

    # New parameters for MongoDB host and port
    mongodb_host = LaunchConfiguration('mongodb_host', default='localhost')
    mongodb_port = LaunchConfiguration('mongodb_port', default=os.getenv('MONGOD_PORT', '27017'))

    return LaunchDescription([
        # Declare Launch Arguments for the existing parameters
        DeclareLaunchArgument(
            'gps_topic', default_value=gps_topic, description='GPS topic for base fix'),
        DeclareLaunchArgument(
            'odometry_topic', default_value=odometry_topic, description='A topic for odometry'),
        DeclareLaunchArgument(
            'battery_status_topic', default_value=battery_status, description='Battery status topic'),
        DeclareLaunchArgument(
            'estop_status_topic', default_value=estop_status, description='E-stop status topic'),
        DeclareLaunchArgument(
            'hunter_status_topic', default_value=hunter_status, description='hunter status topic'),
        DeclareLaunchArgument(
            'actioned_by_coordinator_topic', default_value=actioned_by_coordinator_topic, description=''),
        
        DeclareLaunchArgument(
            'min_distance_threshold', default_value=min_distance_threshold, description=''),
        
        DeclareLaunchArgument(
            'aoc_scenario_path', default_value=aoc_scenario_path, description='path to aoc scenario'),
        DeclareLaunchArgument(
            'aoc_navigation_path', default_value=aoc_navigation_path, description='path to aoc navigation'),

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
                'odometry_topic': odometry_topic,
                'battery_status_topic': battery_status,
                'estop_status_topic': estop_status,
                'hunter_status_topic': hunter_status,
                'aoc_scenario_path': aoc_scenario_path,
                'aoc_navigation_path': aoc_navigation_path,
                'mongodb_host': mongodb_host,
                'mongodb_port': mongodb_port,
            }],
        )
    ])
