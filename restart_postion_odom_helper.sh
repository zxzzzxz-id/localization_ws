#!/bin/bash
pkill -f 'LiDAR_Driver' || true
pkill -f 'FAST_LIO' || true
pkill -f 'Serial_Node' || true
pkill -f 'ros2 launch livox_ros_driver2 msg_MID360_launch.py' || true
pkill -f 'ros2 launch fast_lio mapping_mid360.launch.py' || true
# Stop a node started before the package rename as well, so it cannot retain the port.
pkill -f 'ros2 run tf_to_serial tf_to_serial_node' || true
sleep 1

exec "$@"
