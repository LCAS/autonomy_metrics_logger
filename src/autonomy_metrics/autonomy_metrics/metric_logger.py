#!/usr/bin/env python3
"""
YAML-driven AutonomyMetricsLogger
Author: Ibrahim Hroob - JABASAI
Updated: Fixed Battery Pub & Added Intervention Snapshot
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

from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr


def import_msg_type(type_str: str):
    """Dynamically imports message types based on string."""
    try:
        parts = type_str.split('/')
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

        # Parameters
        self.declare_parameter('config_yaml', '/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/jabas/autonomy_metrics_logger/src/autonomy_metrics/config/metrics_full.yaml')
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27018)
        self.declare_parameter('remote_mongodb_host', '')
        self.declare_parameter('remote_mongodb_port', 27017)
        self.declare_parameter('enable_remote_logging', False)    
        self.declare_parameter('min_distance_threshold', 0.2)
        self.declare_parameter('stop_timeout', 2.0) 

        self.config_path = self.get_parameter('config_yaml').get_parameter_value().string_value
        self.mongo_host = self.get_parameter('mongodb_host').get_parameter_value().string_value
        self.mongo_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
        self.remote_mongo_host = self.get_parameter('remote_mongodb_host').get_parameter_value().string_value
        self.remote_mongo_port = self.get_parameter('remote_mongodb_port').get_parameter_value().integer_value
        self.enable_remote_logging = self.get_parameter('enable_remote_logging').get_parameter_value().bool_value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').get_parameter_value().double_value
        self.stop_timeout = self.get_parameter('stop_timeout').get_parameter_value().double_value

        self.get_logger().info(f"Config path: {self.config_path}")

        # DB Managers
        self.db_mgr_local = DBMgr(host=self.mongo_host, port=self.mongo_port)
        self.db_mgr_remote = None
        if self.enable_remote_logging and self.remote_mongo_host:
            try:
                self.db_mgr_remote = DBMgr(host=self.remote_mongo_host, port=self.remote_mongo_port)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize remote DB manager: {e}")

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
        
        # ----------------------------------------------------
        # 1. NEW: Global Snapshot Storage
        # Stores the latest extracted values from all topics
        # ----------------------------------------------------
        self.system_snapshot = {}
        
        # Snapshot of latest values per field for change detection
        self.prev_field_values = {}

        # Current battery level (updated from any topic with 'battery_field')
        self.current_battery = None       
        
        # Odometry / Speed state
        self.previous_x = None
        self.previous_y = None
        self.init_pose = True
        self.speed = 0.0
        self.last_odom_update_time = self.get_clock().now()

        self.dynamic_subs = []
        self.dynamic_publishers = {}
        self.topic_cfg_map = {}

        # Static Publishers (Standard Metrics)
        self.heartbeat_publisher = self.create_publisher(Bool, 'mdbi_logger/heartbeat', 10)
        self.distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_traveled_distance', 10)
        self.incidents_publisher = self.create_publisher(Int8, 'mdbi_logger/total_incidents', 10)
        self.speed_publisher = self.create_publisher(Float32, 'mdbi_logger/robot_speed', 10)
                
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

        self.db_mgr_local.init_session(env_variables, git_repos)
        if self.db_mgr_remote:
            self.db_mgr_remote.init_session(env_variables, git_repos)
            
    def load_and_setup_config(self):
        if not self.config_path:
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
            
            if not name or not type_str: continue

            self.topic_cfg_map[name] = item

            # Dynamic Publisher Setup
            pub_cfg = item.get('publish', {})
            if pub_cfg.get('enable', False):
                try:
                    pub_msg_cls = import_msg_type(pub_cfg['type'])
                    pub = self.create_publisher(pub_msg_cls, pub_cfg['topic'], 10)
                    self.dynamic_publishers[name] = (pub, pub_msg_cls, pub_cfg.get('field'))
                    self.get_logger().info(f"Dynamic Pub: {pub_cfg['topic']} -> {name}")
                except Exception as e:
                    self.get_logger().error(f"Pub creation failed for {name}: {e}")
                    self.dynamic_publishers[name] = None
            else:
                self.dynamic_publishers[name] = None

            # Subscription Setup
            try:
                msg_cls = import_msg_type(type_str)
                role = item.get('role', '').lower()

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

    def get_metrics_snapshot(self):
        """
        Returns a lightweight metrics summary to attach to every event.
        """
        metrics = {
            "distance": self.distance,
            "autonomous_distance": self.autonomous_distance,
            "speed": self.speed,
        }
        if self.current_battery is not None:
            metrics["battery_percentage"] = self.current_battery
        return metrics

    def trigger_intervention(self, event_type: str, extra: dict | None = None):
        """
        Centralised intervention logic:
        - increments incidents
        - adds system snapshot
        - logs event with metrics
        """
        if extra is None:
            extra = {}

        self.incidents += 1

        base_details = dict(self.details)  # estop, operation_mode, etc.
        base_details.update(extra)
        base_details["system_snapshot"] = self.system_snapshot.copy()

        self.log_event(event_type, base_details)

    def handle_intervention_triggers(self, topic_name: str, data: dict, cfg: dict):
        """
        Apply YAML-configured triggers for this topic.
        - intervention_on_message: triggers on any message
        - intervention_on_change[field]: triggers when selected field changes
        """
        # 1) Trigger on any message (e.g. /cmd_vel/joy)
        msg_trig = cfg.get("intervention_on_message", {})
        if msg_trig.get("enable", False):
            evt_type = msg_trig.get("event_type", f"{topic_name}_activity")
            self.trigger_intervention(evt_type, extra={"topic": topic_name})

        # 2) Trigger on change of specific fields
        field_trigs = cfg.get("intervention_on_change", {})
        for field_name, trig_cfg in field_trigs.items():
            if field_name not in data:
                continue

            key = (topic_name, field_name)
            prev = self.prev_field_values.get(key)
            new = data[field_name]
            changed = (prev is None) or (prev != new)
            self.prev_field_values[key] = new

            if not changed:
                continue

            # Optional: only trigger on a specific value (e.g. True)
            trigger_value = trig_cfg.get("trigger_value", None)
            if trigger_value is not None and new != trigger_value:
                continue

            evt_type = trig_cfg.get(
                "event_type", f"{topic_name}:{field_name}_changed"
            )
            extra = {
                "topic": topic_name,
                "field": field_name,
                "new_value": new,
                "prev_value": prev,
            }
            self.trigger_intervention(evt_type, extra=extra)

    # -------------------------
    # Callbacks
    # -------------------------
    def odom_role_callback(self, topic_name, msg):
        try:
            pos = msg.pose.pose.position
            # Update Snapshot with raw odom info
            self.system_snapshot['odometry'] = {
                'x': pos.x,
                'y': pos.y,
                'vx': msg.twist.twist.linear.x
            }
        except Exception:
            return

        current_time = self.get_clock().now()

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

        if dist < self.min_distance_threshold:
            return

        if not hasattr(self, 'previous_time'):
            self.previous_time = current_time

        time_diff = (current_time - self.previous_time).nanoseconds * 1e-9

        self.speed = dist / time_diff if time_diff > 0 else 0.0
        self.distance += dist
        if self.details.get('operation_mode') == self.AUTO:
            self.autonomous_distance += dist

        self.previous_x = pos.x
        self.previous_y = pos.y
        self.previous_time = current_time
        self.last_odom_update_time = current_time

        # NEW: keep metrics in the snapshot too
        self.system_snapshot['metrics'] = {
            'distance': self.distance,
            'autonomous_distance': self.autonomous_distance,
            'speed': self.speed,
        }

        self.publish_distance(self.distance)
        self.publish_speed(self.speed)
        self.handle_dynamic_publish(topic_name, msg)


    def control_mode_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        mode_field = cfg.get('mode_field', 'data')
        try:
            val = get_nested_field(msg, mode_field)
        except Exception:
            return

        mapping = cfg.get('mode_mapping', {})
        new_mode = mapping.get(str(val)) or mapping.get(val)

        if new_mode is None:
            if str(val) == "3":
                new_mode = self.MAN
            else:
                new_mode = self.AUTO

        current_time = datetime.now()

        if new_mode != self.details.get('operation_mode'):
            prev_mode = self.details.get('operation_mode')
            self.details['operation_mode'] = new_mode
            self.get_logger().info(f"Mode changed: {prev_mode} -> {new_mode}")

            if prev_mode == self.AUTO and new_mode == self.MAN:
                if self.autonomous_start_time:
                    delta = (current_time - self.autonomous_start_time).total_seconds()
                    self.autonomous_time += delta
                    self.autonomous_start_time = None

                # Manual override is an intervention
                self.trigger_intervention('Manual_override')

            elif new_mode == self.AUTO:
                self.autonomous_start_time = current_time
                # Logging autonomous mode change (not counted as intervention)
                self.log_event('Autonomous', {
                    **self.details,
                    'system_snapshot': self.system_snapshot.copy()
                })


    def estop_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        field = cfg.get('field', 'data')
        try:
            v = bool(get_nested_field(msg, field))
            # Update Snapshot
            self.system_snapshot['estop'] = v
        except Exception:
            return

        if v != self.details.get('estop', False):
            self.details['estop'] = v
            self.get_logger().info(f"E-Stop: {v}")
            if v:
                # E-stop counts as intervention
                self.trigger_intervention('EMS')


    def generic_callback(self, topic_name, msg, log_fields):
        cfg = self.topic_cfg_map.get(topic_name, {})
        data = {}

        # Extract data specified in log_fields
        for f in log_fields:
            try:
                data[f] = get_nested_field(msg, f)
            except Exception:
                pass

        if data:
            # Update Snapshot with the latest values from this topic
            self.system_snapshot[topic_name] = data

            # If this topic carries battery info, update the battery state
            battery_field = cfg.get("battery_field")
            if battery_field and battery_field in data:
                self.current_battery = data[battery_field]

            # Topic-level event logging
            event = {
                'time': datetime.now(tz=timezone.utc),
                'event_type': f"topic:{topic_name}",
                'details': data
            }

            self.db_mgr_local.add_event(event)
            if self.db_mgr_remote:
                try:
                    self.db_mgr_remote.add_event(event)
                except Exception:
                    pass

        # Apply YAML-configured intervention triggers (message & field-change)
        self.handle_intervention_triggers(topic_name, data, cfg)

        # Dynamic republishing if configured
        self.handle_dynamic_publish(topic_name, msg)


    def handle_dynamic_publish(self, topic_name, msg):
        pub_cfg = self.dynamic_publishers.get(topic_name)
        if not pub_cfg: return
        publisher, pub_msg_cls, pub_field = pub_cfg
        
        try:
            val = get_nested_field(msg, pub_field) if pub_field else msg
            
            out_msg = pub_msg_cls()
            if hasattr(out_msg, 'data'):
                # ---------------------------------------------------------
                # 4. FIX: Robust Type Casting
                # If target is Float32 but val is int, simple assignment works
                # but explicit cast is safer.
                # ---------------------------------------------------------
                if 'Float' in pub_msg_cls.__name__:
                    out_msg.data = float(val)
                elif 'Int' in pub_msg_cls.__name__:
                    out_msg.data = int(val)
                elif 'String' in pub_msg_cls.__name__:
                    out_msg.data = str(val)
                else:
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
        
        except Exception as e:
            # ---------------------------------------------------------
            # 5. FIX: Log Errors
            # Don't fail silently. Tell user why it failed.
            # ---------------------------------------------------------
            self.get_logger().error(f"Dynamic publish failed for {topic_name}: {e}")

    # -------------------------
    # Timer & Logging
    # -------------------------
    def timer_callback(self):
        hb = Bool()
        hb.data = True
        self.heartbeat_publisher.publish(hb)

        now = self.get_clock().now()
        time_since_move = (now - self.last_odom_update_time).nanoseconds * 1e-9
        
        if time_since_move > self.stop_timeout and self.speed > 0.0:
            self.speed = 0.0
            self.publish_speed(0.0)

    def update_db_metrics(self):
        mdbi_val = float(self.distance) / float(self.incidents) if self.incidents != 0 else self.distance
        db_managers = [self.db_mgr_local]
        if self.db_mgr_remote:
            db_managers.append(self.db_mgr_remote)

        for dbm in db_managers:
            try:
                dbm.update_distance(self.distance)
                dbm.update_incidents(self.incidents)
                dbm.update_autonomous_distance(self.autonomous_distance)
                dbm.update_mdbi(mdbi_val)
            except Exception as e:
                self.get_logger().warn(f"DB update failed: {e}")

    def log_event(self, msg='', details=None):
        if details is None:
            details = {}

        # Attach metrics to every event
        details = {
            **details,
            'metrics': self.get_metrics_snapshot()
        }

        event_time = datetime.now(tz=timezone.utc)
        event = {'time': event_time, 'event_type': msg, 'details': details}

        self.db_mgr_local.add_event(event)
        if self.db_mgr_remote:
            try:
                self.db_mgr_remote.add_event(event)
            except Exception:
                pass

        incidents_msg = Int8()
        incidents_msg.data = self.incidents
        self.incidents_publisher.publish(incidents_msg)

        self.update_db_metrics()
        

    def publish_distance(self, dist):
        msg = Float32()
        msg.data = float(dist)
        self.distance_publisher.publish(msg)
        self.update_db_metrics()

    def publish_speed(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.speed_publisher.publish(msg)

    def get_git_info(self, repo_path):
        """
        Return basic git metadata for a repo.

        Schema (example):
        {
            "path": "/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario",
            "exists": True,
            "remote": "git@github.com:LCAS/aoc_strawberry_scenario.git",
            "branch": "main",
            "commit": "f3a6c1b4e6f8c3b3c823f2c45e56a123456789ab",
            "short_commit": "f3a6c1b",
            "dirty": false,
            "error": null
        }
        """
        info = {
            "path": repo_path,
            "exists": False,
            "remote": None,
            "branch": None,
            "commit": None,
            "short_commit": None,
            "dirty": None,
            "error": None,
        }

        try:
            if not repo_path or not os.path.isdir(repo_path):
                info["error"] = "Path does not exist or is not a directory"
                return info

            def git(args):
                result = subprocess.run(
                    ["git", "-C", repo_path] + args,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip())
                return result.stdout.strip()

            info["exists"] = True

            # Remote (best effort)
            try:
                info["remote"] = git(["config", "--get", "remote.origin.url"])
            except Exception:
                info["remote"] = None

            # Branch (HEAD might be detached)
            try:
                info["branch"] = git(["rev-parse", "--abbrev-ref", "HEAD"])
            except Exception:
                info["branch"] = None

            # Commit hashes
            try:
                info["commit"] = git(["rev-parse", "HEAD"])
            except Exception:
                info["commit"] = None

            try:
                info["short_commit"] = git(["rev-parse", "--short", "HEAD"])
            except Exception:
                info["short_commit"] = None

            # Dirty / clean state
            try:
                status = git(["status", "--porcelain"])
                info["dirty"] = bool(status)
            except Exception:
                info["dirty"] = None

        except Exception as e:
            info["error"] = str(e)

        return info


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