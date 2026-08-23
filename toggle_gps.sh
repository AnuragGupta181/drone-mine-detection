#!/bin/bash
# Helper script to toggle GPS mode for simulation presentation vs strict testing

if [ "$1" == "off" ] || [ "$1" == "false" ]; then
    export PX4_PARAM_EKF2_GPS_CTRL=0
    sed -i 's/GPS_ENABLED=true/GPS_ENABLED=false/' /home/ubuntu/px4_ros2_ws/.env
    echo "========================================="
    echo " MODE: Strict GPS-Denied Testing"
    echo " EKF2_GPS_CTRL set to 0"
    echo "========================================="
else
    export PX4_PARAM_EKF2_GPS_CTRL=7
    sed -i 's/GPS_ENABLED=false/GPS_ENABLED=true/' /home/ubuntu/px4_ros2_ws/.env
    echo "========================================="
    echo " MODE: Smooth Presentation / Video Recording"
    echo " EKF2_GPS_CTRL set to 7 (GPS Enabled)"
    echo "========================================="
fi
