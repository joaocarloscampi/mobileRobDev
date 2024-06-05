#!/bin/bash
for (( ; ; ))
do
	ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
	sleep 30
done
