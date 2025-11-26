#!/usr/bin/env python3
"""
YAML-driven AutonomyMetricsLogger
Author: Ibrahim Hroob (adapted)
"""

import os
import math
import subprocess
import yaml
from datetime import datetime, timezone
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# keep a few explicit imports used by default publishers (std msgs)
from std_msgs.msg import Bool, Float32, Int8, String

# Optional: existing DB manager
from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr

# Helper: dynamic message import
def import_msg_type(type_str: str):
    """
    Accepts strings like 'sensor_msgs/msg/NavSatFix' and returns the message class.
    """
    try:
        parts = type_str.split('/')
        if len(parts) != 3:
            raise ValueError("Message type must be like 'pkg/msg/MessageName'")
        pkg, submodule, cls_name = parts
        module = import_module(f"{pkg}.{submodule}")
        return getattr(module, cls_name)
    except Exception as e:
        raise ImportError(f"Failed to import message type '{type_str}': {e}")

# Helper: get nested attribute (supports attributes and list indices)
def get_nested_field(obj, path: str):
    """
    path examples:
      - 'pose.pose.position.x'
      - 'status.status'
      - 'position_covariance[0]'
    """
    try:
        cur = obj
        for part in path.split('.'):
            # support index like field[0]
            if '[' in part and part.endswith(']'):
                name, idx = part[:-1].split('[')
                cur = getattr(cur, name)
                cur = cur[int(idx)]
            else:
                cur = getattr(cur, part)
        return cur
    except Exception as e:
        raise AttributeError(f"Failed to get '{path}': {e}")

class AutonomyMetricsLogger(Node):
    def __init__(self):
        super().__init__('mdbi_logger_dynamic')

        # Basic state
        self.get_logger().info("Starting YAML-driven AutonomyMetricsLogger")

        # Default parameters
        self.declare_parameter('config_yaml', '')
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27017)
        self.declare_parameter('min_distance_threshold', 0.2)

        # Read parameters
        self.config_path = self.get_parameter('config_yaml').get_parameter_value().string_value
        self.mongo_host = self.get_parameter('mongodb_host').get_parameter_value().string_value
        self.mongo_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').get_parameter_value().double_value

        self.get_logger().info(f"Config path: {self.config_path}")
        self.get_logger().info(f"Mongo host: {self.mongo_host}, port: {self.mongo_port}")
        self.get_logger().info(f"min_distance_threshold: {self.min_distance_threshold}")

        # Database manager
        self.db_mgr = DBMgr(host=self.mongo_host, port=self.mongo_port)

        # Internal metrics state (keep similar to your original node)
        self.mdbi = None
        self.incidents = 0
        self.distance = 0.0
        self.autonomous_distance = 0.0
        self.autonomous_time = 0.0
        self.autonomous_start_time = None
        self.AUTO = 'Autonomous'
        self.MAN = 'Manual'
        self.details = {'estop': False, 'operation_mode': self.MAN}
        self.previous_x = None
        self.previous_y = None
        self.init_pose = True
        self.first_gps_fix_received = False
        self.prev_speed = 0.0
        self.speed = 0.0

        # dynamic structures
        self.dynamic_subs = []
        self.dynamic_publishers = {}  # keyed by topic_name -> (publisher, message_class, publish_field)
        self.topic_cfg_map = {}  # keyed by topic_name -> cfg dict

        # Default static publishers (distance/speed/incidents/heartbeat)
        self.heartbeat_publisher = self.create_publisher(Bool, 'mdbi_logger/heartbeat', 10)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_traveled_distance', 10)
        self.incidents_publisher = self.create_publisher(Int8, 'mdbi_logger/total_incidents', 10)
        self.speed_publisher = self.create_publisher(Float32, 'mdbi_logger/robot_speed', 10)
        self.battery_publisher = self.create_publisher(Float32, 'mdbi_logger/battery_level', 10)

        # load yaml config and create dynamic subs/pubs
        self.load_and_setup_config()

        # initialize DB session
        env_variables = {
            'robot_name': os.getenv('ROBOT_NAME', 'UNDEFINED'),
            'farm_name': os.getenv('FARM_NAME', 'UNDEFINED'),
            'field_name': os.getenv('FIELD_NAME', 'UNDEFINED'),
            'application': os.getenv('APPLICATION', 'UNDEFINED'),
            'scenario_name': os.getenv('SCENARIO_NAME', 'UNDEFINED'),
        }
        # try to capture git info if provided in config
        git_repos = []
        try:
            git_cfg = self.config.get('git_repos', {})
            for label, p in git_cfg.items():
                git_repos.append({label: self.get_git_info(p)})
        except Exception:
            pass

        self.db_mgr.init_session(env_variables, git_repos)

    # -------------------------
    # Config loading & setup
    # -------------------------
    def load_and_setup_config(self):
        # Load YAML
        if not self.config_path:
            raise RuntimeError("Parameter 'config_yaml' must be set to a YAML file path")

        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f) or {}

        topics = self.config.get('topics', [])
        if not isinstance(topics, list):
            raise RuntimeError("'topics' must be a list in YAML config")

        for item in topics:
            name = item.get('name')
            type_str = item.get('type')
            if not name or not type_str:
                self.get_logger().warning(f"Skipping invalid topic config: {item}")
                continue

            # store cfg
            self.topic_cfg_map[name] = item

            # create publisher if requested
            pub_cfg = item.get('publish', {})
            if pub_cfg.get('enable', False):
                try:
                    pub_msg_cls = import_msg_type(pub_cfg['type'])
                    pub_topic = pub_cfg['topic']
                    publisher = self.create_publisher(pub_msg_cls, pub_topic, 10)
                    self.dynamic_publishers[name] = (publisher, pub_msg_cls, pub_cfg.get('field'))
                except Exception as e:
                    self.get_logger().error(f"Failed to create publisher for {name}: {e}")
                    self.dynamic_publishers[name] = None
            else:
                self.dynamic_publishers[name] = None

            # create subscription
            try:
                msg_cls = import_msg_type(type_str)
            except Exception as e:
                self.get_logger().error(f"Failed to import msg type {type_str} for topic {name}: {e}")
                continue

            # Determine if this topic has a special role
            role = item.get('role', '').lower()
            if role == 'odometry':
                # special odometry callback to compute distances
                sub_cb = lambda msg, tn=name: self.odom_role_callback(tn, msg)
            elif role == 'control_mode':
                sub_cb = lambda msg, tn=name: self.control_mode_role_callback(tn, msg)
            elif role == 'estop':
                sub_cb = lambda msg, tn=name: self.estop_role_callback(tn, msg)
            else:
                # generic callback
                log_fields = item.get('log_fields', [])
                sub_cb = lambda msg, tn=name, lf=log_fields: self.generic_callback(tn, msg, lf)

            self.create_subscription(msg_cls, name, sub_cb, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribed to {name} as {type_str} (role={role})")

    # -------------------------
    # Role-specific callbacks
    # -------------------------
    def odom_role_callback(self, topic_name, msg):
        # Expect msg is nav_msgs/msg/Odometry
        try:
            pos = msg.pose.pose.position
        except Exception as e:
            self.get_logger().error(f"Odometry callback failed to extract position: {e}")
            return

        # Generic logging of configured fields
        cfg = self.topic_cfg_map.get(topic_name, {})
        for f in cfg.get('log_fields', []):
            try:
                v = get_nested_field(msg, f)
                # attach to details and log a DB event if requested
                self.details.setdefault('odom_fields', {})[f] = v
            except Exception:
                self.get_logger().debug(f"Could not read odom field {f}")

        # distance initialization
        if self.init_pose:
            self.init_pose = False
            self.previous_x = pos.x
            self.previous_y = pos.y
            return

        dx = pos.x - self.previous_x
        dy = pos.y - self.previous_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.min_distance_threshold:
            return

        # compute speed (time based)
        current_time = self.get_clock().now()
        if hasattr(self, 'previous_time'):
            time_diff = (current_time - self.previous_time).nanoseconds * 1e-9
            self.speed = dist / time_diff if time_diff > 0 else 0.0
        else:
            self.speed = 0.0
        self.previous_time = current_time

        self.distance += dist
        if self.details.get('operation_mode') == self.AUTO:
            self.autonomous_distance += dist

        self.previous_x = pos.x
        self.previous_y = pos.y

        # publish metrics
        self.publish_distance(self.distance)
        self.publish_speed(self.speed)

        # optionally publish a configured publisher mirroring some field
        pub_cfg = self.dynamic_publishers.get(topic_name)
        if pub_cfg:
            publisher, pub_msg_cls, pub_field = pub_cfg
            if pub_field:
                try:
                    v = get_nested_field(msg, pub_field)
                    out_msg = pub_msg_cls()
                    # Common case: std_msgs with .data
                    if hasattr(out_msg, 'data'):
                        out_msg.data = v
                        publisher.publish(out_msg)
                    else:
                        # naive assign if same attribute name
                        for attr in dir(out_msg):
                            if not attr.startswith('_'):
                                try:
                                    setattr(out_msg, attr, v)
                                    publisher.publish(out_msg)
                                    break
                                except Exception:
                                    continue
                except Exception as e:
                    self.get_logger().debug(f"Publish transform failed for {topic_name}: {e}")

    def control_mode_role_callback(self, topic_name, msg):
        # Expect control mode to be a message containing a field that maps to operation mode string or integer
        cfg = self.topic_cfg_map.get(topic_name, {})
        mode_field = cfg.get('mode_field', 'data')  # default to std_msgs/String .data
        try:
            val = get_nested_field(msg, mode_field)
        except Exception as e:
            self.get_logger().error(f"Control mode: failed to read field {mode_field}: {e}")
            return

        # interpret val: try to map ints to modes if mapping exists
        mapping = cfg.get('mode_mapping', {})  # e.g. {"3": "Manual", "1": "Autonomous", "0": "Autonomous"}
        new_mode = None
        if isinstance(val, (int, float)):
            new_mode = mapping.get(str(int(val)), None)
        elif isinstance(val, str):
            new_mode = mapping.get(val, val)
        else:
            new_mode = str(val)

        if new_mode is None:
            # fallback: if val is int 3 -> MAN else -> AUTO
            if isinstance(val, (int, float)) and int(val) == 3:
                new_mode = self.MAN
            else:
                new_mode = self.AUTO

        # emulate previous control_mode_callback behaviour
        current_time = datetime.now()
        if new_mode != self.details.get('operation_mode'):
            previous_mode = self.details.get('operation_mode')
            self.details['operation_mode'] = new_mode
            self.get_logger().info(f"operation_mode : {new_mode}")

            if previous_mode == self.AUTO and new_mode == self.MAN:
                if self.autonomous_start_time is not None:
                    elapsed_autonomous_time = (current_time - self.autonomous_start_time).total_seconds()
                    self.autonomous_time += elapsed_autonomous_time
                    self.details['autonomous_time'] = self.autonomous_time
                    self.autonomous_start_time = None

            elif new_mode == self.AUTO:
                self.autonomous_start_time = current_time

            if new_mode == self.MAN:
                self.incidents += 1
                self.log_event('Manual_override', self.details)
            elif new_mode == self.AUTO:
                self.log_event('Autonomous', self.details)

    def estop_role_callback(self, topic_name, msg):
        # default expect Bool-like .data
        cfg = self.topic_cfg_map.get(topic_name, {})
        estop_field = cfg.get('field', 'data')
        try:
            v = get_nested_field(msg, estop_field)
        except Exception as e:
            self.get_logger().error(f"Estop: failed to read {estop_field}: {e}")
            return

        if bool(v) != bool(self.details.get('estop', False)):
            self.details['estop'] = bool(v)
            self.get_logger().info(f"Estop status changed to: {self.details['estop']}")
            if self.details['estop']:
                self.incidents += 1
                self.log_event('EMS', self.details)

    # -------------------------
    # Generic callback
    # -------------------------
    def generic_callback(self, topic_name, msg, log_fields):
        # Log configured fields
        data = {}
        for f in log_fields or []:
            try:
                v = get_nested_field(msg, f)
                data[f] = v
            except Exception as e:
                self.get_logger().debug(f"generic_callback: failed to read {f} from {topic_name}: {e}")

        if data:
            event = {
                'time': datetime.now(tz=timezone.utc),
                'event_type': f"topic:{topic_name}",
                'details': data
            }
            self.db_mgr.add_event(event)

        # Optional publisher configured for this topic
        pub_cfg = self.dynamic_publishers.get(topic_name)
        if pub_cfg:
            publisher, pub_msg_cls, pub_field = pub_cfg
            try:
                val = get_nested_field(msg, pub_field)
                out_msg = pub_msg_cls()
                if hasattr(out_msg, 'data'):
                    out_msg.data = val
                    publisher.publish(out_msg)
                else:
                    # attempt shallow assignment of first writable attribute
                    for attr in dir(out_msg):
                        if not attr.startswith('_'):
                            try:
                                setattr(out_msg, attr, val)
                                publisher.publish(out_msg)
                                break
                            except Exception:
                                continue
            except Exception as e:
                self.get_logger().debug(f"generic_callback publish failed for {topic_name}: {e}")

    # -------------------------
    # Logging & DB updates
    # -------------------------
    def log_event(self, msg='', details=None):
        if details is None:
            details = {}
        event_time = datetime.now(tz=timezone.utc)
        event = {'time': event_time, 'event_type': msg, 'details': details}
        self.db_mgr.add_event(event)

        # publish incidents
        incidents_msg = Int8()
        incidents_msg.data = self.incidents
        self.incidents_publisher.publish(incidents_msg)

        # update DB metric values
        self.db_mgr.update_distance(self.distance)
        self.db_mgr.update_incidents(self.incidents)
        self.db_mgr.update_autonomous_distance(self.autonomous_distance)
        self.mdbi = float(self.distance) / float(self.incidents) if self.incidents != 0 else 0.0
        self.db_mgr.update_mdbi(self.mdbi)

    # -------------------------
    # Heartbeat & metrics publishing
    # -------------------------
    def publish_heartbeat(self):
        hb = Bool()
        hb.data = True
        self.heartbeat_publisher.publish(hb)
        # If speed unchanged, publish zero to indicate stop
        if self.prev_speed != self.speed:
            self.prev_speed = self.speed
        else:
            self.publish_speed(0.0)

    def publish_distance(self, distance):
        dmsg = Float32()
        dmsg.data = float(distance)
        self.distance_publisher.publish(dmsg)
        self.db_mgr.update_distance(self.distance)
        self.db_mgr.update_autonomous_distance(self.autonomous_distance)
        self.get_logger().debug(f"Distance published: {self.distance}")

    def publish_speed(self, speed):
        smsg = Float32()
        smsg.data = float(speed)
        self.speed_publisher.publish(smsg)

    # -------------------------
    # Git info helper (optional)
    # -------------------------
    def get_git_info(self, repo_path="."):
        try:
            commit_hash = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo_path).decode().strip()
            commit_message = subprocess.check_output(["git", "log", "-1", "--pretty=%B"], cwd=repo_path).decode().strip()
            commit_author = subprocess.check_output(["git", "log", "-1", "--pretty=%an"], cwd=repo_path).decode().strip()
            commit_date = subprocess.check_output(["git", "log", "-1", "--pretty=%ct"], cwd=repo_path).decode().strip()
            commit_date = datetime.fromtimestamp(int(commit_date)).strftime('%Y-%m-%d %H:%M:%S')
            branch_name = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=repo_path).decode().strip()
            try:
                remote_url = subprocess.check_output(["git", "config", "--get", "remote.origin.url"], cwd=repo_path).decode().strip()
            except subprocess.CalledProcessError:
                remote_url = "No remote"
            return {
                "build_version": commit_hash[:7],
                "commit_message": commit_message,
                "commit_author": commit_author,
                "commit_date": commit_date,
                "branch_name": branch_name,
                "remote_url": remote_url
            }
        except Exception:
            return {
                "build_version": "Invalid repository",
                "commit_message": "",
                "commit_author": "",
                "commit_date": "",
                "branch_name": "Unknown",
                "remote_url": "Unknown"
            }

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyMetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
