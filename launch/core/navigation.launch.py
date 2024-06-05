import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	# Diretórios e arquivos
	bringup_dir = get_package_share_directory('robovisor')

	# Argumentos de lançamento
	map_file = LaunchConfiguration('map')
	map_file_arg = DeclareLaunchArgument('map',
										default_value=os.path.join(bringup_dir, 'maps/dc_2_zlac/dc_2_zlac.yaml'),
										description='Full path to map file to load')
	
	use_sim_time = LaunchConfiguration('use_sim_time')
	sim_time_arg = DeclareLaunchArgument('use_sim_time',
										default_value='false',
										description='Use simulation (Gazebo) clock if true')
	
	params_file = LaunchConfiguration('params_file')
	params_file_arg = DeclareLaunchArgument('params_file',
										default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
										description='Full path to the ROS2 parameters file to use for all launched nodes')
	
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	rviz_config_file_arg = DeclareLaunchArgument('rviz_config_file',
										default_value=os.path.join(bringup_dir, 'rviz', 'nav2.rviz'),
										description='Full path to the RVIZ config file to use')

	# Launches
	publisher = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([FindPackageShare('robovisor'), '/launch/core/publisher.launch.py']),
		launch_arguments={'use_sim_time': use_sim_time, 'rviz_config_file': rviz_config_file}.items()
	)

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch', '/bringup_launch.py']),
		launch_arguments={'map': map_file, 'use_sim_time': use_sim_time, 'params_file': params_file}.items()
	)

	return LaunchDescription([
		map_file_arg,
		params_file_arg,
		sim_time_arg,
		rviz_config_file_arg,
		publisher,
		nav2
	])