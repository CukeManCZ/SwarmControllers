#!/bin/bash

TOPIC="/boidConstants"
MSG_TYPE="myc_messages/msg/KeyValueFloatArray"

echo "Interactive Boid Constants Publisher"
echo "Enter key and value separated by space (e.g., flocking 1.5)"
echo "Valid keys: flocking, avoidance, velocity, waypoint"
echo "Type 'q' to quit"

while true; do
    read -p "Enter key and value: " key value

    if [[ "$key" == "q" ]]; then
        echo "Exiting..."
        break
    fi

    # Validate value is a number
    if ! [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
        echo "Invalid value. Must be a number."
        continue
    fi

    msg_yaml="data:
  - key: \"$key\"
    value: $value"

    echo "Publishing $key = $value on $TOPIC"

    # Publish once
    ros2 topic pub --once $TOPIC $MSG_TYPE "$msg_yaml"

    echo "-----------------------------------"
done
