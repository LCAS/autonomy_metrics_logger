from pymongo import MongoClient
from datetime import datetime, timezone
from bson import ObjectId

# NEW
try:
    import numpy as np
except Exception:
    np = None


class DatabaseMgr:
    """
    A class to manage database operations for robot session.
    """

    def __init__(self, database_name='robot_incidents', host='localhost', port=27017):
        self.client = MongoClient(
            f'mongodb://{host}:{port}/',
            connectTimeoutMS=None
        )
        self.db = self.client[database_name]
        self.sessions_collection = self.db['sessions']
        self.session_id = None

    # NEW
    def _bson_safe(self, obj):
        """Recursively convert non-BSON types (notably numpy) into Mongo-friendly types."""
        if np is not None:
            # numpy arrays -> list
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            # numpy scalar -> python scalar
            if isinstance(obj, np.generic):
                return obj.item()

        if isinstance(obj, dict):
            return {k: self._bson_safe(v) for k, v in obj.items()}

        if isinstance(obj, (list, tuple)):
            return [self._bson_safe(v) for v in obj]

        return obj

    def init_session(self, env_variables, aoc_repos_info):
        session_start = datetime.now(tz=timezone.utc)
        session_document = {
            "session_start_time": session_start,
            "robot_name": env_variables['robot_name'],
            "farm_name": env_variables['farm_name'],
            "field_name": env_variables['field_name'],
            "application": env_variables['application'],
            "scenario_name": env_variables['scenario_name'],
            "aoc_repos_info": aoc_repos_info,
            "mdbi": None,
            "incidents": 0,
            "distance": 0,
            "autonomous_distance": 0,
            "collision_incidents": 0,
            "events": []
        }

        # NEW: sanitize (in case aoc_repos_info contains numpy types)
        session_document = self._bson_safe(session_document)

        result = self.sessions_collection.insert_one(session_document)
        self.session_id = result.inserted_id

    def add_event(self, event):
        # NEW: sanitize event before writing
        event = self._bson_safe(event)

        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$addToSet": {"events": event}}
        )
        return result.modified_count > 0

    def update_incidents(self, incidents):
        incidents = self._bson_safe(incidents)  # harmless; keeps pattern consistent
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"incidents": incidents}}
        )
        return result.modified_count > 0

    def update_distance(self, distance):
        distance = self._bson_safe(distance)
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"distance": distance}}
        )
        return result.modified_count > 0

    def update_autonomous_distance(self, autonomous_distance):
        autonomous_distance = self._bson_safe(autonomous_distance)
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"autonomous_distance": autonomous_distance}}
        )
        return result.modified_count > 0

    def update_mdbi(self, mdbi):
        mdbi = self._bson_safe(mdbi)
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"mdbi": mdbi}}
        )
        return result.modified_count > 0

    def update_collision_incidents(self, collision_incidents):
        collision_incidents = self._bson_safe(collision_incidents)
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {"collision_incidents": collision_incidents}}
        )
        return result.modified_count > 0
