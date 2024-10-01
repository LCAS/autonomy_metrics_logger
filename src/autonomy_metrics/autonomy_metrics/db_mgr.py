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

from pymongo import MongoClient
from datetime import datetime, timezone
from bson import ObjectId

class DatabaseMgr:
    """
    A class to manage database operations for robot session.

    Attributes:
        client (MongoClient): The MongoDB client.
        db (Database): The MongoDB database.
        sessions_collection (Collection): The MongoDB collection for sessions.
        session_id (ObjectId): The ID of the current session.
    """

    def __init__(self, database_name='robot_incidents', host='localhost', port=27017):
        """
        Initializes the DatabaseMgr with a given database name, host, and port.

        Args:
            database_name (str): The name of the database to connect to.
            host (str): The hostname or IP address of the MongoDB server.
            port (int): The port of the MongoDB server.
        """
        self.client = MongoClient(f'mongodb://{host}:{port}/')
        self.db = self.client[database_name]
        self.sessions_collection = self.db['sessions']
        self.session_id = None

    def init_session(self, env_variables, aoc_repos_info):
        """
        Initializes a new session with the given environmental variables.

        Args:
            **env_variables: Arbitrary keyword arguments containing session information.
                Expected keys include 'robot_name', 'farm_name', 'field_name', 'application',
                and 'scenario_name'.
        """
        session_start = datetime.now(tz=timezone.utc)
        session_document = {
            "session_start_time": session_start,
            "robot_name": env_variables['robot_name'],
            "farm_name": env_variables['farm_name'],
            "field_name": env_variables['field_name'],
            "application": env_variables['application'],
            "scenario_name": env_variables['scenario_name'],
            "aoc_repos_info": aoc_repos_info, 
            "incidents": 0,
            "distance": 0,
            "autonomous_distance": 0,
            "events": []
        }
        result = self.sessions_collection.insert_one(session_document)
        self.session_id = result.inserted_id

    def add_event(self, event):
        """
        Adds an event to the current session.

        Args:
            event (dict): The event to add.

        Returns:
            bool: True if the event was added successfully, False otherwise.
        """
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$addToSet": {"events": event}}
        )
        return result.modified_count > 0

    def update_incidents(self, incidents):
        """
        Updates the incidents value of the current session.

        Args:
            incidents (int): The new incidents value.

        Returns:
            bool: True if the incidents was updated successfully, False otherwise.
        """
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"incidents": incidents}}
        )
        return result.modified_count > 0

    def update_distance(self, distance):
        """
        Updates the distance value of the current session.

        Args:
            distance (float): The new distance value.

        Returns:
            bool: True if the distance was updated successfully, False otherwise.
        """
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"distance": distance}}
        )
        return result.modified_count > 0

    def update_autonomous_distance(self, autonomous_distance):
        """
        Updates the autonomous_distance value of the current session.

        Args:
            autonomous_distance (float): The new autonomous_distance value.

        Returns:
            bool: True if the autonomous_distance was updated successfully, False otherwise.
        """
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"autonomous_distance": autonomous_distance}}
        )
        return result.modified_count > 0