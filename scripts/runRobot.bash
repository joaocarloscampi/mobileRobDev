#!/bin/bash

readonly CAMERA="camera_start.bash";
chmod +x $CAMERA;
gnome-terminal -x bash -c "./$CAMERA; exec $SHELL"

source ~/yolo_ws/install/setup.bash
ros2 launch robovisor yolo_general.launch.py


