# MDBI Logger Node

MDBI Logger is a ROS2 node designed to log robot events and calculate the Mean Distance Between Incidents (MDBI). The node subscribes to various topics to gather data such as GPS coordinates, battery status, and emergency stop status, and logs these events to a database.

## Features

- **Calculate MDBI**: Computes the mean distance between incidents based on the robot's GPS data.
- **Event Logging**: Logs significant events such as emergency stops and operation mode changes.
- **Database Integration**: Stores event logs and distance metrics in a database.


## Usage

1. **Set Environment Variables:**

    Set the following environment variables to configure your robot's session:

    ```sh
    export ROBOT_NAME=<your-robot-name>
    export FARM_NAME=<your-farm-name>
    export FIELD_NAME=<your-field-name>
    export APPLICATION=<your-application>
    export SCENARIO_NAME=<your-scenario-name>
    ```

2. **Run the Node:**

    ```sh
    TODO
    ```

## Parameters

The node uses the following parameters, which can be overridden:

- `gps_topic` (default: `/gps_base/fix`)
- `gps_odom_topic` (default: `/gps_base/odometry`)
- `battery_status` (default: `/battery_status`)
- `estop_status` (default: `/estop_status`)

## Node Details

### Subscriptions

- **GPS Data** (`/gps_base/fix`): Receives GPS data of the robot.
- **Odometry Data** (`/gps_base/odometry`): Receives odometry data for distance calculation.
- **Battery Status** (`/battery_status`): Monitors the battery level.
- **Emergency Stop Status** (`/estop_status`): Monitors the emergency stop status.


## License

Licensed under the Apache License, Version 2.0. See the [LICENSE](http://www.apache.org/licenses/LICENSE-2.0) file for more details.
