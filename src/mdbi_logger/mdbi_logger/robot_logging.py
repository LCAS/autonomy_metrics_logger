import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter 


from pymongo import MongoClient
from datetime import datetime, timezone
from bson import ObjectId


class MDBILogger(Node):
    """A class for logging robot events to calculate MDBI"""
    def __init__(self):
        super().__init__('mdbi_logger')

        # those may need to be in a service ? 
        self.declare_parameter('robot_name'    , rclpy.Parameter.Type.STRING)
        self.declare_parameter('environment'   , rclpy.Parameter.Type.STRING)

        self.declare_parameter('gps_topic'     , rclpy.Parameter.Type.STRING)
        self.declare_parameter('gps_odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('driver_topic'  , rclpy.Parameter.Type.STRING)


        self.robot_name  = self.get_parameter_or('robot_name' , Parameter('str', Parameter.Type.STRING, 'dogtooth')).value
        self.environment = self.get_parameter_or('environment', Parameter('str', Parameter.Type.STRING, 'poly_tunnel_riseholme')).value
        
        self.gps_topic      = self.get_parameter_or('gps_topic'     , Parameter('str', Parameter.Type.STRING, '/gps')).value
        self.gps_odom_topic = self.get_parameter_or('gps_odom_topic', Parameter('str', Parameter.Type.STRING, '/gps_odom')).value
        self.driver_topic   = self.get_parameter_or('driver_topic'  , Parameter('str', Parameter.Type.STRING, '/driver')).value
        
        self.travelled_distance = 0
        self.mdbi = 0

        self.get_logger().info('Initializing logging node')

        # init mongo client and create database
        self.client = MongoClient('mongodb://localhost:27017/')
        self.db = self.client['mdbi_logs']
        self.sessions_collection = self.db['sessions']

        # Log a session
        self.session_id = self.__log_session()
        self.get_logger().info(f"Session logged with ID: {self.session_id}")

        # Add a start event to the existing session
        new_event = self.__log_event(0, {"lat": 0, "lon": 0}, "start")
        self.__add_event_to_session(new_event)

        self.__add_event_to_session(self.__log_event(1, {"lat": 1, "lon": 1}, "estop"))

        self.__add_event_to_session(self.__log_event(10, {"lat": 40, "lon": 40}, "finish"))

        # subscribe to the topics 


    def __log_session(self):
        session_start = datetime.now(tz=timezone.utc)
        session_document = {
            "robot_name": self.robot_name,
            "environment": self.environment,
            "session_start": session_start,
            "MDBI": 0,
            "events": []           
        }

        result = self.sessions_collection.insert_one(session_document)
        return result.inserted_id
    
    def __log_event(self, traveled_distance, gps_location, event_type):
        return {
            "traveled_distance": traveled_distance,
            "gps_location": gps_location,
            "time_of_event": datetime.now(tz=timezone.utc),
            "event_type": event_type
        }       

    def __add_event_to_session(self, event):
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$addToSet": {"events": event}}
        )

        if result.modified_count > 0:
            self.get_logger().info(f"New unique event added to session with ID: {self.session_id}")
        else:
            self.get_logger().info(f"Event already exists or failed to add event to session with ID: {self.session_id}")

    def __update_mdbi_name(self, mdbi):
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"robot_name": mdbi}}
        )

        if result.modified_count > 0:
            self.get_logger().info(f"MDBI updated to {mdbi} for session with ID: {self.session_id}")
        else:
            self.get_logger().info(f"Failed to update MDBI for session with ID: {self.session_id}")



if __name__ == "__main__":
    rclpy.init() 
    md = MDBILogger()
