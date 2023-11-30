#!/bin/bash

# Function to handle the SIGINT signal
cleanup() {
    echo -e "\033[0;32mStopping rosbag and roslaunch...\033[0m"
    kill -INT "$ROSBAG_PID"
    kill -INT "$ROSLAUNCH_PID"
    exit 0
}
source devel/setup.bash
# Run roslaunch
roslaunch spot_fsm_control spot_fsm_control.launch &
ROSLAUNCH_PID=$!
echo -e "\033[0;32mRoslaunch started with PID: $ROSLAUNCH_PID\033[0m"

# Run rosbag
rosbag record /compressed_data /gaze_hit_object /hand_keypoints &
ROSBAG_PID=$!
echo -e "\033[0;32mRosbag started with PID: $ROSBAG_PID\033[0m"

# Trap SIGINT signal and call cleanup function
trap 'cleanup' SIGINT

# Wait for 'q' key press
while : ; do
    read -n 1 k <&1
    if [[ $k = q ]] ; then
        printf "\033[0;32m\nKey 'q' pressed. Exiting...\033[0m\n"
        cleanup
    fi
done
