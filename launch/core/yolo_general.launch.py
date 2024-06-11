import os
import xacro
import yaml
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
	
	map_saver_params_file = 'map_saver_params.yaml'
	map_save_config = os.path.join(bringup_dir, 'config', map_saver_params_file)

	# AMCL parametros
	map_yaml_file = os.path.join(bringup_dir, 'maps', 'yolo_maps', 'lidar', 'map_1_lidar.yaml')
	params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

	robot_config_file = os.path.join(bringup_dir, 'config/robot_parameters.yaml')
	with open(robot_config_file, 'r') as file:
		robotParams = yaml.safe_load(file)
	print(robotParams)
	
	robot = robotParams['robot']
	zed_scan = robotParams['zed_scan']
	zed_camera_model = robotParams['zed_camera_model']
	lidar = robotParams['lidar']
	filter_lidar = robotParams['filter_lidar']
	slam_toolbox = robotParams['slam_toolbox']
	teleop_keyboard = robotParams['teleop_keyboard']
	rviz_enable = robotParams['rviz']
	amcl_localization = robotParams['amcl_localization']
	
	print("Robot: ", robot)
	print("Zed scan enable: ", zed_scan)
	print("Zed model: : ", zed_camera_model)
	print("Lidar enable: ", lidar)
	print("Lidar filter enable: ", filter_lidar)
	print("slam_toolbox enable: ", slam_toolbox)
	
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

	joint_state_publiser_node = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		output='screen',
		#condition=UnlessCondition(simulation)
	)
	
	list_launch_description = [use_sim_time_arg, robot_state_publisher_node, joint_state_publiser_node]
	
	# Launches
	if robot=='zlac':
		driver = Node(
			package='robovisor',
			executable='zlac',
			output='screen'
		)
	elif robot=='movebase':
		driver = Node(
			package='robovisor',
			executable='robovisor_node',
			output='screen'
		)
	else:
		print("Parametro robot errado")
	list_launch_description.append(driver)

	if slam_toolbox:
		rviz_config_file=os.path.join(bringup_dir, 'rviz', 'nav2_yolo.rviz')
	elif amcl_localization:
		rviz_enable = False # Abrir RViz na mao (por enquanto)
	else:
		rviz_config_file=os.path.join(bringup_dir, 'rviz', 'view.rviz')
	
	if rviz_enable:
		rviz_node = Node(
			package='rviz2',
			executable='rviz2',
			output='screen',
			parameters=[{'use_sim_time': False}],
			arguments=['-d', rviz_config_file]
		)
		list_launch_description.append(rviz_node)

	if slam_toolbox:
		map_server = Node(
			package='nav2_map_server',
			executable='map_saver_server',
			output='screen',
			emulate_tty=True,  
			parameters=[map_save_config]                  
		)

		lifecycle_manager = Node(
			package='nav2_lifecycle_manager',
			executable='lifecycle_manager',
			name='lifecycle_manager',
			output='screen',
			emulate_tty=True,  
			parameters=[
				{'use_sim_time': False},
				{'autostart': True},
				{'node_names': ['map_saver']}]
		)

		slam = IncludeLaunchDescription(
			PythonLaunchDescriptionSource([FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py']),
			launch_arguments={'use_sim_time': use_sim_time, 'params_file': os.path.join(bringup_dir, 'config/mapper_params_online_async_yolo.yaml')}.items()
		)
		list_launch_description.append(map_server)
		list_launch_description.append(lifecycle_manager)
		list_launch_description.append(slam)
	
	if amcl_localization:
		nav2 = IncludeLaunchDescription(
			PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch', '/localization_launch.py']),
			launch_arguments={'map': map_yaml_file, 'use_sim_time': 'false', 'params_file': params_file}.items()
		)
		list_launch_description.append(nav2)

	if lidar:
		if filter_lidar:
			filter_lidar = 'true'
		else:
			filter_lidar = 'false'
			
		rplidar = IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch/real/rplidar.launch.py')),
			launch_arguments={'rplidar_filter': filter_lidar}.items()
		)
		list_launch_description.append(rplidar)
	
	if teleop_keyboard:
		teleop = Node(
			package='teleop_twist_keyboard',
			executable="teleop_twist_keyboard",
			output='screen', 
			prefix = 'xterm -e'
			)
		list_launch_description.append(teleop)

	return LaunchDescription(list_launch_description)
