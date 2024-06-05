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

	# Diretórios e arquivos
	bringup_dir = get_package_share_directory('robovisor')

	# Argumentos de Lançamento
	config = LaunchConfiguration('config')
	config_arg = DeclareLaunchArgument('config',
										default_value=os.path.join(bringup_dir, 'config', 'joy.yaml'),
										description='Path to config files')

	joy_dev = LaunchConfiguration('joy_dev')
	joy_dev_arg = DeclareLaunchArgument('joy_dev',
										default_value='/dev/input/js0',
										description='Device to use')

	# Launches
	joy_node = Node(
		package='joy',
		executable='joy_node',
		output='screen',
		parameters=[{
			'dev': joy_dev,
			'deadzone': 0.3,
			'autorepeat_rate': 20.0
		}]
	)

	teleop_node = Node(
		package='teleop_twist_joy',
		executable='teleop_node',
		output='screen',
		parameters=[config]
	)

	return LaunchDescription([
		config_arg,
		joy_dev_arg,
		joy_node,
		teleop_node
	])