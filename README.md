# Autonomy Metrics Logger Node

The Autonomy Metrics Logger is a ROS2 node that tracks various robot events and computes the **Mean Distance Between Incidents (MDBI)**. This node monitors topics such as GPS, odometry, battery status, and emergency stop status to log incidents and key metrics into a MongoDB database.

## Features

- **MDBI Calculation**: Computes the Mean Distance Between Incidents based on traveled distance and logged incidents.
- **Autonomous Mode Tracking**: Records the time spent and distance covered in autonomous mode.
- **Event Logging**: Logs critical events like emergency stops, manual overrides, and coordinator tasks.
- **Database Integration**: Uses MongoDB to store event logs, distances, and operational details of the robot.
- **Heartbeat Signal**: Publishes a heartbeat message every 5 seconds to indicate the node is alive.
- **Git Repositories Logging**: Captures and stores Git information of the active repositories for traceability.

## Usage

1. **Set Environment Variables:**

   Configure the robot session by setting the following environment variables:

    ```sh
    export ROBOT_NAME=<your-robot-name>
    export FARM_NAME=<your-farm-name>
    export FIELD_NAME=<your-field-name>
    export APPLICATION=<your-application>
    export SCENARIO_NAME=<your-scenario-name>
    ```

2. **Run the Node:**

    ```sh
    ros2 launch autonomy_metrics autonomy_metrics.launch.py
    ```

## Parameters

The node uses the following ROS parameters, which can be customized:

- `mongodb_host` (default: `localhost`): MongoDB server host.
- `mongodb_port` (default: `27017`): MongoDB server port.
- `aoc_scenario_path` (default: `""`): Path to the AOC scenario repository.
- `aoc_navigation_path` (default: `""`): Path to the AOC navigation repository.
- `min_distance_threshold` (default: `0.2`): Minimum distance threshold for recording traveled distance (in meters).
- `gps_topic` (default: `/gps_base/fix`)
- `odometry_topic` (default: `/diff_drive_controller/odom`)
- `battery_status_topic` (default: `/diff_drive_controller/battery_status`)
- `estop_status_topic` (default: `/diff_drive_controller/estop_status`)
- `hunter_status_topic` (default: `/hunter_status`)
- `actioned_by_coordinator_topic` (default: `/topological_navigation/execute_policy_mode/goal`)

## Node Details

### Subscriptions

- **GPS Data** (`/gps_base/fix`): Receives and logs GPS data of the robot.
- **Odometry Data** (`/diff_drive_controller/odom`): Monitors odometry data to calculate traveled distance.
- **Battery Status** (`/diff_drive_controller/battery_status`): Logs battery level information.
- **Emergency Stop Status** (`/diff_drive_controller/estop_status`): Monitors and logs emergency stop events.
- **Hunter Status** (`/hunter_status`): Tracks the robot's current status, including operation mode and battery voltage.
- **Coordinator Task** (`/topological_navigation/execute_policy_mode/goal`): Logs tasks received from the coordinator.

### Publications

- **Heartbeat** (`mdbi_logger/heartbeat`): Publishes a signal every 5 seconds to indicate node activity.

## License

Licensed under the Apache License, Version 2.0. See the [LICENSE](http://www.apache.org/licenses/LICENSE-2.0) file for more details.
