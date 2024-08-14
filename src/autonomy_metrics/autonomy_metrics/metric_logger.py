
"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Author: Ibrahim Hroob
Email: ibrahim.hroub7@gmail.com
"""

import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr
from datetime import datetime, timezone
from rclpy.qos import qos_profile_sensor_data

class AutonomyMetricsLogger(Node):
    """
    A class for logging robot events to calculate MDBI (Mean Distance Between Incidents).

    Attributes:
        mdbi (float): Mean Distance Between Incidents.
        distance (float): Traveled distance in meters.
        db_mgr (DBMgr): Instance of DatabaseMgr for database operations.
        gps_data (dict): GPS data of the robot.
        estop (bool): Emergency stop status.
        battery_level (float): Battery level of the robot.
        operation_mode (str): Operation mode of the robot.
        previous_x (float): Previous x-coordinate for distance calculation.
        previous_y (float): Previous y-coordinate for distance calculation.
        env_variables (dict): Environmental variables for the robot session.
    """

    def __init__(self):
        super().__init__('mdbi_logger')
        self.get_logger().info('Initializing logging node')

        # Define useful global variables
        self.mdbi = 0  # Mean Distance Between Incidents
        self.incidents = 0
        self.distance = 0  # Traveled distance in meters

        # Declare and get the ROS parameters, including MongoDB host and port
        self.declare_and_get_parameters()

        # Initialize DatabaseMgr with the retrieved port
        self.db_mgr = DBMgr(host=self.mongo_host, port=self.mongo_port)

        self.gps_data = None
        self.estop = False
        self.battery_level = 0
        self.operation_mode = None
        self.previous_x = None
        self.previous_y = None
        self.first_gps_fix = True

        # Get robot name and other environmental variables (Group them into a dictionary)
        env_variables = {
            'robot_name': os.getenv('ROBOT_NAME', 'UNDEFINED'),
            'farm_name': os.getenv('FARM_NAME', 'UNDEFINED'),
            'field_name': os.getenv('FIELD_NAME', 'UNDEFINED'),
            'application': os.getenv('APPLICATION', 'UNDEFINED'),
            'scenario_name': os.getenv('SCENARIO_NAME', 'UNDEFINED'),
        }

        # Subscribe to the topics
        self.create_subscriptions()

        # Initialize session in the database
        self.db_mgr.init_session(env_variables)

    def log_event(self, msg=''):
        event_time = datetime.now(tz=timezone.utc)
        event = {
            'time': event_time,
            'msg': msg,
            'estop': self.estop,
            'distance': self.distance,
            'battery_level': self.battery_level,
            'operation_mode': self.operation_mode,
            'gps': self.gps_data, 
        }
        self.db_mgr.add_event(event)
        self.db_mgr.update_distance(self.distance)
        self.db_mgr.update_incidents(self.incidents)

    def declare_and_get_parameters(self):
        # Declare MongoDB host and port as ROS parameters
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27017)

        # Retrieve the parameters
        self.mongo_host = self.get_parameter('mongodb_host').get_parameter_value().string_value
        self.mongo_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value

        param_defaults = {
            'gps_topic': '/gps_base/fix',
            'gps_odom_topic': '/gps_base/odometry',
            'battery_status': '/battery_status',
            'estop_status': '/estop_status'
        }

        self.params = {}
        for param, default in param_defaults.items():
            self.declare_parameter(param, default)
            self.params[param] = self.get_parameter(param).get_parameter_value().string_value

        for key, value in self.params.items():
            self.get_logger().info(f"{key.replace('_', ' ').title()}: {value}")

    def create_subscriptions(self):
        self.create_subscription(Float32, self.params['battery_status'], self.battery_level_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Bool, self.params['estop_status'], self.estop_sub_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self.params['gps_topic'], self.gps_fix_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Odometry, self.params['gps_odom_topic'], self.gps_odom_callback, qos_profile=qos_profile_sensor_data)

    def battery_level_callback(self, msg):
        self.battery_status = msg.data

    def estop_sub_callback(self, msg):
        if msg.data != self.estop:
            self.estop = msg.data
            self.get_logger().info(f"Estop status changed to: {self.estop}")

            if self.estop:
                self.incidents += 1
                self.get_logger().info(f"Incident count incremented to: {self.incidents}")
                self.log_event('EMS')

    def gps_fix_callback(self, msg):
        self.gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': list(msg.position_covariance),
            'position_covariance_type': msg.position_covariance_type,
            'status': msg.status.status,
            'service': msg.status.service
        }

        if self.first_gps_fix:
            self.first_gps_fix = False
            self.log_event('First_GNSS_msg')

    def gps_odom_callback(self, msg):
        position = msg.pose.pose.position

        # Calculate the traveled distance
        if self.previous_x is not None and self.previous_y is not None:
            dx = position.x - self.previous_x
            dy = position.y - self.previous_y
            distance = math.sqrt(dx**2 + dy**2)
            self.distance += distance

        self.previous_x = position.x
        self.previous_y = position.y


def main(args=None):
    rclpy.init(args=args)
    
    autonomy_metric_logger = AutonomyMetricsLogger()
    
    rclpy.spin(autonomy_metric_logger)
    autonomy_metric_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
