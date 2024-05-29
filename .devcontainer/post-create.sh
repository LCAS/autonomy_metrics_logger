#!/bin/bash

function add_config_if_not_exist {
    if ! grep -F -q "$1" $HOME/.bashrc; then
        echo "$1" >> $HOME/.bashrc
    fi
}

add_config_if_not_exist "source /opt/ros/humble/setup.bash"
LOCAL_SETUP_FILE=`pwd`/install/setup.bash
add_config_if_not_exist "if [ -r $LOCAL_SETUP_FILE ]; then source $LOCAL_SETUP_FILE; fi"

source /opt/ros/humble/setup.bash
colcon build --symlink-install --continue-on-error


# Manually Starting MongoDB
## Context
# In containerized environments, starting MongoDB with `systemctl` or `systemd` is often 
# not feasible, as these environments might not utilize `systemd` for service management.
# Consequently, MongoDB must be started manually in such cases.

echo "Starting mongodb manually.."

db_dir=/workspaces/mdbi_logger/src/mdbi_logger/data/db

# Run MongoDB in the Background
nohup mongod --dbpath $db_dir > /dev/null 2>&1 &

# Verify MongoDB is Running
if ps aux | grep -v grep | grep mongod > /dev/null
then
    echo -e "\e[32mMongod is initialized and running successfully\e[0m"
else
    echo -e "\e[31mMongod process not found\e[0m"
fi
