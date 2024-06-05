import os
import xacro
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

	# Argumentos de Lançamento
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	rviz_config_file_arg = DeclareLaunchArgument('rviz_config_file', 
											default_value=os.path.join(bringup_dir, 'rviz', 'view.rviz'), 
											description='Full path to the RVIZ config file to use. Available: nav2, view, slam')
	
	simulation = LaunchConfiguration('simulation')
	simulation_arg = DeclareLaunchArgument('simulation',
											default_value='false',
											description='Use simulation if true')


	# Processar arquivo xacro para urdf
	xacro_file = os.path.join(bringup_dir, 'urdf/robo_dc/robo_dc.urdf.xacro')
	doc = xacro.process_file(xacro_file)
	urdf = doc.toxml()

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': urdf}]
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		output='screen',
		parameters=[{'use_sim_time': False}],
		arguments=['-d', rviz_config_file]
	)

	joint_state_publiser_node = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		output='screen',
		#condition=UnlessCondition(simulation)
	)

	driver = Node(
		package='robovisor',
		executable='zlac',
		output='screen',
		#condition=UnlessCondition(simulation)
	)

	rplidar = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch/real/rplidar.launch.py')),
		#condition=UnlessCondition(simulation)
	)
	
	
	# ZED Wrapper node
	#zed_wrapper_launch = IncludeLaunchDescription(
	#	PythonLaunchDescriptionSource([get_package_share_directory('zed_wrapper'), '/launch/include/zed_camera.launch.py']),
	#	launch_arguments={
	#		'camera_model': 'zedm',
	#		'camera_name': 'zedm',
	#		'publish_urdf': False,
	#		'publish_tf':False
	#	}.items()
	#)


	return LaunchDescription([
		rviz_config_file_arg,
		robot_state_publisher_node,
		rviz_node,
		joint_state_publiser_node,
		driver,
		rplidar
		#zed_wrapper_launch
	])
