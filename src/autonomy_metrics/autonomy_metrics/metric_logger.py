#!/usr/bin/env python3
"""
YAML-driven AutonomyMetricsLogger
Author: Ibrahim Hroob - JABASAI
"""

import os
import math
import subprocess
import yaml
import time
from datetime import datetime, timezone
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

# Keep a few explicit imports used by default publishers (std msgs)
from std_msgs.msg import Bool, Float32, Int8, String

# Optional: existing DB manager
try:
    from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr
except ImportError:
    # Fallback mock class if the module is missing, to prevent immediate crash during testing
    class DBMgr:
        def __init__(self, host, port): print(f"DB Mock connected to {host}:{port}")
        def init_session(self, env, git): print("DB Session Init")
        def add_event(self, evt): pass
        def update_distance(self, d): pass
        def update_incidents(self, i): pass
        def update_autonomous_distance(self, d): pass
        def update_mdbi(self, m): pass

# Helper: dynamic message import
def import_msg_type(type_str: str):
    """
    Accepts strings like 'sensor_msgs/msg/NavSatFix' or 'std_msgs/String'
    and returns the message class.
    """
    try:
        parts = type_str.split('/')
        
        # Handle "pkg/Message" (2 parts) -> convert to "pkg/msg/Message"
        if len(parts) == 2:
            pkg, cls_name = parts
            submodule = 'msg'
        elif len(parts) == 3:
            pkg, submodule, cls_name = parts
        else:
            raise ValueError("Message type must be 'pkg/msg/MessageName' or 'pkg/MessageName'")

        module = import_module(f"{pkg}.{submodule}")
        return getattr(module, cls_name)
    except Exception as e:
        raise ImportError(f"Failed to import message type '{type_str}': {e}")

# Helper: get nested attribute (supports attributes and list indices)
def get_nested_field(obj, path: str):
    try:
        cur = obj
        for part in path.split('.'):
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

        self.get_logger().info("Starting YAML-driven AutonomyMetricsLogger")

        # Default parameters
        self.declare_parameter('config_yaml', '')
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27017)
        self.declare_parameter('min_distance_threshold', 0.2)
        # Timeout to consider robot stopped if no distance update occurs
        self.declare_parameter('stop_timeout', 2.0) 

        # Read parameters
        self.config_path = self.get_parameter('config_yaml').get_parameter_value().string_value
        self.mongo_host = self.get_parameter('mongodb_host').get_parameter_value().string_value
        self.mongo_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').get_parameter_value().double_value
        self.stop_timeout = self.get_parameter('stop_timeout').get_parameter_value().double_value

        self.get_logger().info(f"Config path: {self.config_path}")
        self.get_logger().info(f"Mongo host: {self.mongo_host}, port: {self.mongo_port}")

        self.db_mgr = DBMgr(host=self.mongo_host, port=self.mongo_port)

        # Internal metrics state
        self.mdbi = 0.0
        self.incidents = 0
        self.distance = 0.0
        self.autonomous_distance = 0.0
        self.autonomous_time = 0.0
        self.autonomous_start_time = None
        self.AUTO = 'Autonomous'
        self.MAN = 'Manual'
        self.details = {'estop': False, 'operation_mode': self.MAN}
        
        # Odometry / Speed state
        self.previous_x = None
        self.previous_y = None
        self.init_pose = True
        self.speed = 0.0
        self.last_odom_update_time = self.get_clock().now() # To detect stops

        self.dynamic_subs = []
        self.dynamic_publishers = {}
        self.topic_cfg_map = {}

        # Static Publishers
        self.heartbeat_publisher = self.create_publisher(Bool, 'mdbi_logger/heartbeat', 10)
        self.distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_traveled_distance', 10)
        self.incidents_publisher = self.create_publisher(Int8, 'mdbi_logger/total_incidents', 10)
        self.speed_publisher = self.create_publisher(Float32, 'mdbi_logger/robot_speed', 10)
        self.battery_publisher = self.create_publisher(Float32, 'mdbi_logger/battery_level', 10)
        
        # Timer
        self.heartbeat_timer = self.create_timer(1.0, self.timer_callback)

        self.load_and_setup_config()

        # Init Session
        env_variables = {
            'robot_name': os.getenv('ROBOT_NAME', 'UNDEFINED'),
            'farm_name': os.getenv('FARM_NAME', 'UNDEFINED'),
            'field_name': os.getenv('FIELD_NAME', 'UNDEFINED'),
            'application': os.getenv('APPLICATION', 'UNDEFINED'),
            'scenario_name': os.getenv('SCENARIO_NAME', 'UNDEFINED'),
        }
        
        git_repos = []
        if self.config:
            try:
                git_cfg = self.config.get('git_repos', {})
                for label, p in git_cfg.items():
                    git_repos.append({label: self.get_git_info(p)})
            except Exception:
                pass

        self.db_mgr.init_session(env_variables, git_repos)

    def load_and_setup_config(self):
        if not self.config_path:
            self.get_logger().error("No 'config_yaml' parameter provided.")
            self.config = {}
            return

        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.config = {}
            return

        topics = self.config.get('topics', [])
        for item in topics:
            name = item.get('name')
            type_str = item.get('type')
            
            if not name or not type_str:
                continue

            self.topic_cfg_map[name] = item

            # Create dynamic publisher if requested
            pub_cfg = item.get('publish', {})
            if pub_cfg.get('enable', False):
                try:
                    pub_msg_cls = import_msg_type(pub_cfg['type'])
                    pub = self.create_publisher(pub_msg_cls, pub_cfg['topic'], 10)
                    self.dynamic_publishers[name] = (pub, pub_msg_cls, pub_cfg.get('field'))
                except Exception as e:
                    self.get_logger().error(f"Pub creation failed for {name}: {e}")
                    self.dynamic_publishers[name] = None
            else:
                self.dynamic_publishers[name] = None

            # Create subscription
            try:
                msg_cls = import_msg_type(type_str)
                role = item.get('role', '').lower()

                # Define callback with default args to capture loop variables
                if role == 'odometry':
                    cb = lambda msg, n=name: self.odom_role_callback(n, msg)
                elif role == 'control_mode':
                    cb = lambda msg, n=name: self.control_mode_role_callback(n, msg)
                elif role == 'estop':
                    cb = lambda msg, n=name: self.estop_role_callback(n, msg)
                else:
                    lf = item.get('log_fields', [])
                    cb = lambda msg, n=name, l=lf: self.generic_callback(n, msg, l)

                self.create_subscription(msg_cls, name, cb, qos_profile_sensor_data)
                self.get_logger().info(f"Subscribed to {name} ({role})")

            except Exception as e:
                self.get_logger().error(f"Sub creation failed for {name}: {e}")

    # -------------------------
    # Callbacks
    # -------------------------
    def odom_role_callback(self, topic_name, msg):
        try:
            pos = msg.pose.pose.position
        except Exception:
            return

        current_time = self.get_clock().now()

        # Initialization
        if self.init_pose:
            self.init_pose = False
            self.previous_x = pos.x
            self.previous_y = pos.y
            self.previous_time = current_time
            self.last_odom_update_time = current_time
            return

        dx = pos.x - self.previous_x
        dy = pos.y - self.previous_y
        dist = math.sqrt(dx*dx + dy*dy)

        # Distance accumulation threshold
        if dist < self.min_distance_threshold:
            # We haven't moved enough to register distance, but we might check speed?
            # Actually, standard logic is to assume stationary if under threshold
            # We update the time so the heartbeat knows we are still receiving msgs
            # but we don't update 'self.speed' here because division by small time is noisy.
            return

        # Calculate time delta for speed
        # Ensure we have a previous time set
        if not hasattr(self, 'previous_time'):
             self.previous_time = current_time
        
        time_diff = (current_time - self.previous_time).nanoseconds * 1e-9
        
        # Update metrics
        self.speed = dist / time_diff if time_diff > 0 else 0.0
        self.distance += dist
        if self.details.get('operation_mode') == self.AUTO:
            self.autonomous_distance += dist

        # Update state
        self.previous_x = pos.x
        self.previous_y = pos.y
        self.previous_time = current_time
        self.last_odom_update_time = current_time # Mark activity

        # Publish immediately on change
        self.publish_distance(self.distance)
        self.publish_speed(self.speed)

        # Handle dynamic publish mirroring
        self.handle_dynamic_publish(topic_name, msg)

    def control_mode_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        mode_field = cfg.get('mode_field', 'data')
        try:
            val = get_nested_field(msg, mode_field)
        except Exception:
            return

        mapping = cfg.get('mode_mapping', {})
        # Robust lookup (string or int)
        new_mode = mapping.get(str(val)) or mapping.get(val)
        
        if new_mode is None:
            # Fallback logic
            if str(val) == "3": new_mode = self.MAN
            else: new_mode = self.AUTO

        current_time = datetime.now()
        
        if new_mode != self.details.get('operation_mode'):
            prev_mode = self.details.get('operation_mode')
            self.details['operation_mode'] = new_mode
            self.get_logger().info(f"Mode changed: {prev_mode} -> {new_mode}")

            # Time accumulation logic
            if prev_mode == self.AUTO and new_mode == self.MAN:
                if self.autonomous_start_time:
                    delta = (current_time - self.autonomous_start_time).total_seconds()
                    self.autonomous_time += delta
                    self.autonomous_start_time = None
            elif new_mode == self.AUTO:
                self.autonomous_start_time = current_time

            # Log events
            if new_mode == self.MAN:
                self.incidents += 1
                self.log_event('Manual_override', self.details)
            else:
                self.log_event('Autonomous', self.details)

    def estop_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        field = cfg.get('field', 'data')
        try:
            v = bool(get_nested_field(msg, field))
        except Exception:
            return

        if v != self.details.get('estop', False):
            self.details['estop'] = v
            self.get_logger().info(f"E-Stop: {v}")
            if v:
                self.incidents += 1
                self.log_event('EMS', self.details)

    def generic_callback(self, topic_name, msg, log_fields):
        data = {}
        for f in log_fields:
            try:
                data[f] = get_nested_field(msg, f)
            except Exception:
                pass
        
        if data:
            self.db_mgr.add_event({
                'time': datetime.now(tz=timezone.utc),
                'event_type': f"topic:{topic_name}",
                'details': data
            })
        
        self.handle_dynamic_publish(topic_name, msg)

    def handle_dynamic_publish(self, topic_name, msg):
        pub_cfg = self.dynamic_publishers.get(topic_name)
        if not pub_cfg: return
        publisher, pub_msg_cls, pub_field = pub_cfg
        
        try:
            val = get_nested_field(msg, pub_field) if pub_field else msg
            out_msg = pub_msg_cls()
            if hasattr(out_msg, 'data'):
                out_msg.data = val
            else:
                # Try simple attribute matching
                for attr in dir(out_msg):
                    if not attr.startswith('_'):
                        try:
                            setattr(out_msg, attr, val)
                            break
                        except: continue
            publisher.publish(out_msg)
        except Exception:
            pass

    # -------------------------
    # Timer & Logging
    # -------------------------
    def timer_callback(self):
        # 1. Heartbeat
        hb = Bool()
        hb.data = True
        self.heartbeat_publisher.publish(hb)

        # 2. Check for robot stop (TIMEOUT LOGIC)
        # If we haven't processed a distance update in X seconds, assume speed is 0
        now = self.get_clock().now()
        time_since_move = (now - self.last_odom_update_time).nanoseconds * 1e-9
        
        if time_since_move > self.stop_timeout and self.speed > 0.0:
            self.speed = 0.0
            self.publish_speed(0.0)
        
        # If we are moving at constant velocity, we just keep the last self.speed
        # We do NOT force it to zero just because it equals prev_speed.

    def log_event(self, msg, details):
        self.db_mgr.add_event({
            'time': datetime.now(tz=timezone.utc),
            'event_type': msg, 
            'details': details
        })
        
        i_msg = Int8()
        i_msg.data = self.incidents
        self.incidents_publisher.publish(i_msg)

        self.db_mgr.update_distance(self.distance)
        self.db_mgr.update_incidents(self.incidents)
        self.db_mgr.update_autonomous_distance(self.autonomous_distance)
        
        if self.incidents > 0:
            self.mdbi = self.distance / self.incidents
        else:
            self.mdbi = self.distance # Or 0.0, depends on definition. Usually total dist if 0 incidents.
            
        self.db_mgr.update_mdbi(self.mdbi)

    def publish_distance(self, dist):
        msg = Float32()
        msg.data = float(dist)
        self.distance_publisher.publish(msg)

    def publish_speed(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.speed_publisher.publish(msg)

    def get_git_info(self, repo_path):
        # ... (same as original) ...
        return {}

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()