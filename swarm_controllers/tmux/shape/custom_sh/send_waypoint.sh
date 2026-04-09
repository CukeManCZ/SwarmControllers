#!/bin/bash

FRAME_ID="simulator_origin"

echo "Interactive Waypoint Publisher"
echo "Enter X Y Z coordinates separated by spaces (type 'q' to quit)"

while true; do
    read -p "Enter waypoint coordinates: " X Y Z

    if [[ "$X" == "q" ]]; then
        echo "Exiting..."
        break
    fi

    if ! [[ "$X" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
         ! [[ "$Y" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
         ! [[ "$Z" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
        echo "Invalid input. Please enter three numbers separated by spaces."
        continue
    fi

    echo "Publishing waypoint: ($X, $Y, $Z) in frame '$FRAME_ID'"

    ros2 topic pub --once /waypoint geometry_msgs/msg/PointStamped \
    "{header: {frame_id: '${FRAME_ID}'} , point: {x: ${X}, y: ${Y}, z: ${Z}}}"

    echo "-----------------------------------"
done
