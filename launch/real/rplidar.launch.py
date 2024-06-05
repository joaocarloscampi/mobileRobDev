import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	rplidar_filter = LaunchConfiguration('rplidar_filter')
	rplidar_filter_arg = DeclareLaunchArgument('rplidar_filter',
											default_value='true',
											description='Filter rplidar if true')

	# Launches
	rplidar_node = Node(
		package='rplidar_ros',
		executable='rplidar_composition',
		output='screen',
		parameters=[{
			'serial_port': '/dev/rplidar',
			'serial_baudrate': 115200,
			'frame_id': 'laser_link',
			'inverted': False,
			'angle_compensate': True,
			'scan_mode': 'Sensitivity'
		}]
	)

	laser_filter_node = Node(
		package='laser_filters',
		executable='scan_to_scan_filter_chain',
		parameters=[
			PathJoinSubstitution([FindPackageShare('robovisor'), 'config', 'laser_filter.yaml'])
		],
		condition=IfCondition(rplidar_filter)
	)

	return LaunchDescription([
		rplidar_filter_arg,
		rplidar_node,
		laser_filter_node
	])
