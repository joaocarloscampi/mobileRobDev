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
										default_value=os.path.join(bringup_dir, 'maps', 'dc_2_zlac/dc_2_zlac.yaml'),
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
	
	slam_params_file = LaunchConfiguration('slam_params_file')
	slam_params_file_arg = DeclareLaunchArgument('slam_params_file',
										default_value=os.path.join(bringup_dir, 'config', 'localization_params.yaml'),
										description='Full path to the ROS2 parameters file to use for SLAM node')

	# Launches
	publisher = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([FindPackageShare('robovisor'), '/launch/core/publisher.launch.py']),
		launch_arguments={'use_sim_time': use_sim_time, 'rviz_config_file': rviz_config_file}.items()
	)

	remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
	lifecycle_nodes = [
		'controller_server',
		'planner_server',
		'behavior_server',
		'waypoint_follower',
		'bt_navigator'
	]

	slam_node = Node(
		package='slam_toolbox',
		executable='localization_slam_toolbox_node',
		name='slam_toolbox',
		output='screen',
		parameters=[slam_params_file],
		remappings=remappings
	)

	nav2_controller_node = Node(
		package='nav2_controller',
		executable='controller_server',
		name='controller_server',
		output='screen',
		parameters=[params_file]
	)

	nav2_planner_node = Node(
		package='nav2_planner',
		executable='planner_server',
		name='planner_server',
		output='screen',
		parameters=[params_file]
	)

	nav2_behaviors_node = Node(
		package='nav2_behaviors',
		executable='behavior_server',
		name='behavior_server',
		output='screen',
		parameters=[params_file]
	)

	nav2_waypoint_follower_node = Node(
		package='nav2_waypoint_follower',
		executable='waypoint_follower',
		name='waypoint_follower',
		output='screen',
		parameters=[params_file]
	)

	nav2_bt_navigator_node = Node(
		package='nav2_bt_navigator',
		executable='bt_navigator',
		name='bt_navigator',
		output='screen',
		parameters=[params_file]
	)

	nav2_lifecycle_manager_node = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager',
		output='screen',
		parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}]
	)

	return LaunchDescription([
		map_file_arg,
		params_file_arg,
		sim_time_arg,
		rviz_config_file_arg,
		slam_params_file_arg,
		publisher,
		slam_node,
		nav2_controller_node,
		nav2_planner_node,
		nav2_behaviors_node,
		nav2_waypoint_follower_node,
		nav2_bt_navigator_node,
		nav2_lifecycle_manager_node
	])