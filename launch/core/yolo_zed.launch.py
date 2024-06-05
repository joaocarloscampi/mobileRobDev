import os
import xacro
import yaml
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	# Diretórios e arquivos
	bringup_dir = get_package_share_directory('robovisor')
											
	robot_config_file = os.path.join(bringup_dir, 'config/robot_parameters.yaml')
	with open(robot_config_file, 'r') as file:
		robotParams = yaml.safe_load(file)
	print(robotParams)
	
	
	zed_scan = robotParams['zed_scan']
	zed_camera_model = robotParams['zed_camera_model']
	
	
	# ZED Wrapper node
	zed_wrapper_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([get_package_share_directory('zed_wrapper'), '/launch/include/zed_camera.launch.py']),
		launch_arguments={
			'camera_model': zed_camera_model,
			'camera_name': zed_camera_model,
			'publish_urdf': 'false',
			'publish_tf':'false'
		}.items()
	)
	
	list_launch_description = [zed_wrapper_launch]
	
	if zed_scan:
		depth_to_laserscan = Node(
			package='depthimage_to_laserscan',
			executable='depthimage_to_laserscan_node',
			output='screen',
			remappings=[
		        	('/depth', '/zed/zed_node/depth/depth_registered'),
		        	('/depth_camera_info', '/zed/zed_node/depth/camera_info')
		        	#('/scan', '/scan_zedm')
		    	],
			parameters=[{
				'output_frame':"base_link" 
			}]
			
		)
		list_launch_description.append(depth_to_laserscan)


	return LaunchDescription(list_launch_description)
