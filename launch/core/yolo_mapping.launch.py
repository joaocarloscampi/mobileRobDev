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
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_sim_time_arg = DeclareLaunchArgument('use_sim_time', 
											default_value='false', 
											description='Use simulation (Gazebo) clock if true')
	

	rviz_config_file = LaunchConfiguration('rviz_config_file')
	rviz_config_file_arg = DeclareLaunchArgument('rviz_config_file',
											default_value=os.path.join(bringup_dir, 'rviz', 'nav2_yolo.rviz'),
											description='Full path to the RVIZ config file to use. Available: nav2, view, slam')
										
	rplidar_on = LaunchConfiguration('rplidar_on')
	rplidar_on_arg = DeclareLaunchArgument('rplidar_on',
											default_value='true',
											description='Use rplidar if true')

	# Launches
	#publisher = IncludeLaunchDescription(
	#	PythonLaunchDescriptionSource([FindPackageShare('robovisor'), '/launch/core/publisher.launch.py']),
	#	launch_arguments={'use_sim_time': use_sim_time, 'rviz_config_file': rviz_config_file}.items()
	#)
	
    # Launches
	publisher = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([FindPackageShare('robovisor'), '/launch/core/yolo_robot.launch.py']),
		launch_arguments={'rplidar_on':rplidar_on, 'rviz_config_file': rviz_config_file}.items()
	)

	mapping = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py']),
		launch_arguments={'use_sim_time': use_sim_time, 'params_file': os.path.join(bringup_dir, 'config/mapper_params_online_async_yolo.yaml')}.items()
	)

	return LaunchDescription([
		use_sim_time_arg,
		rviz_config_file_arg,
		rplidar_on_arg,
		publisher,
		mapping
	])
