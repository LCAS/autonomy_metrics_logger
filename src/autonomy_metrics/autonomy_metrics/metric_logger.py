
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
import subprocess
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Bool, Int8, String
from nav_msgs.msg import Odometry
from topological_navigation_msgs.msg import ExecutePolicyModeGoal
from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr
from datetime import datetime, timezone
from rclpy.qos import qos_profile_sensor_data

# Try to import HunterStatus and handle if it is unavailable
try:
    from hunter_msgs.msg import HunterStatus
    HUNTER_MSG_AVAILABLE = True
except ImportError:
    HUNTER_MSG_AVAILABLE = False
    print("Warning: hunter_msgs.HunterStatus is not available. Hunter status callbacks will be disabled.")

# Try to import DogroothStatus and handle if it is unavailable
try:
    from dogtooth_msgs.msg import DogtoothStatus
    DOGTOOTH_MSG_AVAILABLE = True
except ImportError:
    DOGTOOTH_MSG_AVAILABLE = False
    print("Warning: hunter_msgs.HunterStatus is not available. Hunter status callbacks will be disabled.")

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
        self.mongo_host = None
        self.mongo_port = None
        self.aoc_scenario_path = ''
        self.aoc_navigation_path = ''
        self.mdbi = None  # Mean Distance Between Incidents
        self.incidents = 0
        self.previous_x = None
        self.previous_y = None
        self.init_pose = True
        self.first_gps_fix_received = False
        self.tasks_received_from_coordinator = 0
        self.distance = 0  # Total traveled distance in meters
        self.autonomous_distance = 0
        self.autonomous_time = 0  # Total time spent in autonomous mode (in seconds)
        self.autonomous_start_time = None  # Time when autonomous mode started
        self.min_distance_threshold = 0.2 #meters
        self.AUTO = 'Autonomous'
        self.MAN = 'Manual'
        self.prev_speed = 0.0
        self.speed = 0.0

        # Init some variables 
        self.details = {}
        self.details['estop'] = False  
        self.details['operation_mode'] = self.MAN

        # Get robot name and other environmental variables (Group them into a dictionary)
        env_variables = {
            'robot_name': os.getenv('ROBOT_NAME', 'UNDEFINED'),
            'farm_name': os.getenv('FARM_NAME', 'UNDEFINED'),
            'field_name': os.getenv('FIELD_NAME', 'UNDEFINED'),
            'application': os.getenv('APPLICATION', 'UNDEFINED'),
            'scenario_name': os.getenv('SCENARIO_NAME', 'UNDEFINED'),
        }
        
        # Declare and get the ROS parameters, including MongoDB host and port
        self.declare_and_get_parameters()

        # Initialize DatabaseMgr with the retrieved port
        self.db_mgr = DBMgr(host=self.mongo_host, port=self.mongo_port)

        # Get aoc git repos info 
        try:
            self.get_logger().info("Retrieving AOC git repository information...")
            aoc_scenario_git_info = self.get_git_info(self.aoc_scenario_path)
        except Exception as e:
            self.get_logger().error(f"Error while logging git info: {e}")
            aoc_scenario_git_info = None

        try:
            self.get_logger().info("Retrieving AOC Navigation git repository information...")
            aoc_navigation_git_info = self.get_git_info(self.aoc_navigation_path)
        except Exception as e:
            self.get_logger().error(f"Error while logging git info: {e}")
            aoc_navigation_git_info = None

        git_repos = []
        git_repos.append({'aoc_scenario_git_info': aoc_scenario_git_info})
        git_repos.append({'aoc_navigation_git_info': aoc_navigation_git_info})

        # Subscribe to the topics
        self.create_subscriptions()

        # Initialize session in the database
        self.db_mgr.init_session(env_variables, git_repos)

        # Heartbeat publisher
        self.heartbeat_publisher = self.create_publisher(Bool, 'mdbi_logger/heartbeat', 10)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)  # Publish every 1 seconds

        # Add publishers for distance, incidents, and speed
        self.distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_traveled_distance', 10)
        self.incidents_publisher = self.create_publisher(Int8, 'mdbi_logger/total_incidents', 10)
        self.speed_publisher = self.create_publisher(Float32, 'mdbi_logger/robot_speed', 10)
        self.battery_publisher = self.create_publisher(Float32, 'mdbi_logger/battery_level', 10)

    def publish_heartbeat(self):
        # Create and publish the heartbeat message
        heartbeat_msg = Bool()
        heartbeat_msg.data = True  # Set to True to indicate the node is alive
        self.heartbeat_publisher.publish(heartbeat_msg)
        self.get_logger().debug('Heartbeat published')

        if self.prev_speed != self.speed:
            self.prev_speed = self.speed
        else:
            self.publish_speed(0.0)

    def declare_and_get_parameters(self):
        # Declare MongoDB host and port as ROS parameters
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27017)
        # Retrieve the parameters
        self.mongo_host = self.get_parameter('mongodb_host').get_parameter_value().string_value
        self.mongo_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
  
        # Log MongoDB host and port
        self.get_logger().info(f"MongoDB Host: {self.mongo_host}")
        self.get_logger().info(f"MongoDB Port: {self.mongo_port}")

        # Declare aoc repos path params
        self.declare_parameter('aoc_scenario_path', '')
        self.declare_parameter('aoc_navigation_path', '')
        # Retrieve the parameters
        self.aoc_scenario_path = self.get_parameter('aoc_scenario_path').get_parameter_value().string_value
        self.aoc_navigation_path = self.get_parameter('aoc_navigation_path').get_parameter_value().string_value

        # Log aoc_scenario_path and aoc_navigation_path
        self.get_logger().info(f"AOC Scenario Path: {self.aoc_scenario_path}")
        self.get_logger().info(f"AOC Navigation Path: {self.aoc_navigation_path}")


        self.declare_parameter('min_distance_threshold', 0.2)
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').get_parameter_value().double_value
        self.get_logger().info(f"min_distance_threshold: {self.min_distance_threshold}")

        param_defaults = {
            'gps_topic': '/gps_base/fix',
            'odometry_topic': '/diff_drive_controller/odom',
            'battery_status_topic': '/diff_drive_controller/battery_status', 
            'estop_status_topic': '/diff_drive_controller/estop_status', 
            'hunter_status_topic': '/hunter_status',  # Change this line
            'dogtooth_status_topic': '/dogtooth_status',  # Change this line
            'actioned_by_coordinator_topic': '/topological_navigation/execute_policy_mode/goal'
        }
        self.params = {}
        for param, default in param_defaults.items():
            self.declare_parameter(param, default)
            self.params[param] = self.get_parameter(param).get_parameter_value().string_value

        for key, value in self.params.items():
            self.get_logger().info(f"{key.replace('_', ' ').title()}: {value}")

    def create_subscriptions(self):
        self.create_subscription(Float32, self.params['battery_status_topic'], self.battery_level_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Bool, self.params['estop_status_topic'], self.estop_sub_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self.params['gps_topic'], self.gps_fix_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Odometry, self.params['odometry_topic'], self.gps_odom_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(ExecutePolicyModeGoal, self.params['actioned_by_coordinator_topic'], self.coordinator_callback, qos_profile=qos_profile_sensor_data) 
        # Only subscribe to hunter status if the message is available
        if HUNTER_MSG_AVAILABLE:
            self.create_subscription(HunterStatus, self.params['hunter_status_topic'], self.hunter_status_callback, qos_profile=qos_profile_sensor_data)
        else:
            self.get_logger().warn("HunterStatus message not available, skipping hunter status subscription.")
        # Only subscribe to dogtooth status if the message is available
        if DOGTOOTH_MSG_AVAILABLE:
            self.create_subscription(DogtoothStatus, self.params['dogtooth_status_topic'], self.dogtooth_status_callback, qos_profile=qos_profile_sensor_data)
        else:
            self.get_logger().warn("DogtoothStatus message not available, skipping dogtooth status subscription.")

    def get_git_info(self, repo_path="."):
        try:
            # Get current commit hash
            commit_hash = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=repo_path
            ).decode().strip()

            # Get current commit message
            commit_message = subprocess.check_output(
                ["git", "log", "-1", "--pretty=%B"], cwd=repo_path
            ).decode().strip()

            # Get current commit author
            commit_author = subprocess.check_output(
                ["git", "log", "-1", "--pretty=%an"], cwd=repo_path
            ).decode().strip()

            # Get current commit date
            commit_date = subprocess.check_output(
                ["git", "log", "-1", "--pretty=%ct"], cwd=repo_path
            ).decode().strip()
            commit_date = datetime.fromtimestamp(int(commit_date)).strftime('%Y-%m-%d %H:%M:%S')

            # Get current branch name
            branch_name = subprocess.check_output(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=repo_path
            ).decode().strip()

            # Get remote URL
            try:
                remote_url = subprocess.check_output(
                    ["git", "config", "--get", "remote.origin.url"], cwd=repo_path
                ).decode().strip()
            except subprocess.CalledProcessError:
                remote_url = "No remote found"

            git_info = {
                "build_version": commit_hash[:7],  # Short hash of the latest commit
                "commit_message": commit_message,  # Commit message
                "commit_author": commit_author,  # Commit author
                "commit_date": commit_date,  # Commit date
                "branch_name": branch_name,  # Current branch name
                "remote_url": remote_url  # Remote URL
            }

            return git_info
        except subprocess.CalledProcessError:
            return {
                "build_version": "Invalid repository",
                "commit_message": "",
                "commit_author": "",
                "commit_date": "",
                "branch_name": "Unknown",
                "remote_url": "Unknown"
            }

    def log_event(self, msg='', details={}):
        event_time = datetime.now(tz=timezone.utc)
        event = {
            'time': event_time,
            'event_type': msg,
            'details': details,
        }
        self.db_mgr.add_event(event)

        # Publish the total number of incidents
        incidents_msg = Int8()
        incidents_msg.data = self.incidents
        self.incidents_publisher.publish(incidents_msg)

        # Also update MDBI and other metrics
        self.db_mgr.update_distance(self.distance)
        self.db_mgr.update_incidents(self.incidents)
        self.db_mgr.update_autonomous_distance(self.autonomous_distance)

        self.mdbi = float(self.distance) / float(self.incidents) if self.incidents != 0 else 0.0
        self.db_mgr.update_mdbi(self.mdbi)

    def battery_level_callback(self, msg):
        self.details['battery_voltage'] = msg.data
        battery_msg = Float32()
        battery_msg.data = msg.data
        self.battery_publisher.publish(battery_msg)
        self.get_logger().debug(f"Battery level published: {msg.data}")

    def estop_sub_callback(self, msg):
        if msg.data != self.details['estop']:
            self.details['estop'] = msg.data
            self.get_logger().info(f"Estop status changed to: {self.details['estop']}")

            if self.details['estop']:
                self.incidents += 1
                self.get_logger().info(f"Incident count incremented to: {self.incidents}")
                self.log_event('EMS', self.details)

    def control_mode_callback(self, msg):
        current_time = datetime.now()
        
        if msg.data != self.details['operation_mode']:
            previous_mode = self.details['operation_mode']
            self.details['operation_mode'] = msg.data
            self.get_logger().info(f"operation_mode : {msg.data}")

            # If switching from AUTO to MANUAL, calculate elapsed time in AUTO mode
            if previous_mode == self.AUTO and self.details['operation_mode'] == self.MAN:
                if self.autonomous_start_time is not None:
                    elapsed_autonomous_time = (current_time - self.autonomous_start_time).total_seconds()
                    self.autonomous_time += elapsed_autonomous_time
                    self.details['autonomous_time'] = self.autonomous_time
                    self.get_logger().info(f"Autonomous mode elapsed time: {elapsed_autonomous_time} seconds")
                    self.autonomous_start_time = None  # Reset the start time

            # If switching to AUTO mode, start timing
            elif self.details['operation_mode'] == self.AUTO:
                self.autonomous_start_time = current_time
                self.get_logger().info(f"Autonomous mode started at: {self.autonomous_start_time}")

            # Log the event
            if self.details['operation_mode'] == self.MAN:
                self.incidents += 1
                self.log_event('Manual_override', self.details)
            elif self.details['operation_mode'] == self.AUTO:
                self.log_event('Autonomous', self.details)

    def coordinator_callback(self, msg):
        edge_id = msg.route.edge_id
        source = msg.route.source
        self.tasks_received_from_coordinator += 1
        self.get_logger().info(f"Task count incremented to: {self.tasks_received_from_coordinator}")
        coordinator = {'edge_id': edge_id, 'source': source, 'tasks_received_from_coordinator': self.tasks_received_from_coordinator}
        self.details['coordinator'] = coordinator
        self.log_event('Coordinator_task', self.details)
        
    def hunter_status_callback(self, msg):
        # Serialize HunterStatus into a dictionary
        serialized_data = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id
            },
            'linear_velocity': msg.linear_velocity,
            'steering_angle': msg.steering_angle,
            'vehicle_state': msg.vehicle_state,
            'control_mode': msg.control_mode,
            'error_code': msg.error_code,
            'battery_voltage': msg.battery_voltage,
            'actuator_states': [
                {
                    'motor_id': actuator.motor_id,
                    'rpm': actuator.rpm,
                    'current': actuator.current,
                    'pulse_count': actuator.pulse_count,
                    'driver_voltage': actuator.driver_voltage,
                    'driver_temperature': actuator.driver_temperature,
                    'motor_temperature': actuator.motor_temperature,
                    'driver_state': actuator.driver_state
                } for actuator in msg.actuator_states
            ]
        }
        self.details['hunter_status'] = serialized_data

        battery = Float32()
        battery.data = msg.battery_voltage
        self.battery_level_callback(battery)

        control_mode = String()
        if msg.control_mode == 3: #this is manual mode in hunter
            control_mode.data = self.MAN
        elif msg.control_mode == 1 or msg.control_mode == 0: #this is auto mode in hunter
            control_mode.data = self.AUTO
        self.control_mode_callback(control_mode)

    def dogtooth_status_callback(self, msg):
        # Serialize HunterStatus into a dictionary
        serialized_data = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id
            },
            'linear_velocity': msg.linear_velocity,
            'angular_velocity': msg.angular_velocity,
            'vehicle_state': msg.vehicle_state,
            'control_mode': msg.control_mode,
            'error_code': msg.error_code,
            'battery_voltage': msg.battery_voltage,
        }
        self.details['dogtooth_status'] = serialized_data

        battery = Float32()
        battery.data = msg.battery_voltage
        self.battery_level_callback(battery)

        control_mode = String()
        if msg.control_mode == 3: #this is manual mode in hunter
            control_mode.data = self.MAN
        elif msg.control_mode == 1 or msg.control_mode == 0: #this is auto mode in hunter
            control_mode.data = self.AUTO
        self.control_mode_callback(control_mode)

    def gps_fix_callback(self, msg):
        gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': list(msg.position_covariance),
            'position_covariance_type': msg.position_covariance_type,
            'status': msg.status.status,
            'service': msg.status.service
        }

        self.details['gps_data'] = gps_data

        if not self.first_gps_fix_received:
            self.first_gps_fix_received = True
            self.log_event('First_GNSS_msg', self.details)

    def gps_odom_callback(self, msg):
        position = msg.pose.pose.position

        if self._initialize_pose(position):
            return

        distance = self._calculate_distance(position)

        if self._is_significant_distance(distance):
            self._update_distances(distance, position)

    def _initialize_pose(self, position):
        if self.init_pose:
            self.init_pose = False
            self.previous_x = position.x
            self.previous_y = position.y
            return True
        return False

    def _calculate_distance(self, position):
        dx = position.x - self.previous_x
        dy = position.y - self.previous_y
        return math.sqrt(dx**2 + dy**2)

    def _is_significant_distance(self, distance):
        return distance >= self.min_distance_threshold

    def _update_distances(self, distance, position):
        current_time = self.get_clock().now()

        # Calculate time difference between the last update and now
        if hasattr(self, 'previous_time'):
            time_diff = (current_time - self.previous_time).nanoseconds * 1e-9  # Convert to seconds
            self.speed = distance / time_diff if time_diff > 0 else 0.0
        else:
            self.speed = 0.0

        # Update the previous time
        self.previous_time = current_time

        # Update the total traveled distance
        self.distance += distance
        if self.details['operation_mode'] == self.AUTO:
            self.autonomous_distance += distance
        self.previous_x = position.x
        self.previous_y = position.y

        # Publish the distance and speed
        self.publish_distance(self.distance)
        self.publish_speed(self.speed)

    def publish_distance(self, distance):
        # Publish total traveled distance
        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_publisher.publish(distance_msg)

        self.db_mgr.update_distance(self.distance)
        self.db_mgr.update_autonomous_distance(self.autonomous_distance)

        # Log the speed and distance
        self.get_logger().debug(f"Traveled distance: {self.distance} meters")

    def publish_speed(self, speed):
        # Publish robot speed
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_publisher.publish(speed_msg)


    def _log_distance(self, distance):
        self.get_logger().debug(f"distance: {distance}")



def main(args=None):
    rclpy.init(args=args)
    
    autonomy_metric_logger = AutonomyMetricsLogger()
    
    rclpy.spin(autonomy_metric_logger)
    autonomy_metric_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
