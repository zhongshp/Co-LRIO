from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os.path
import yaml
from launch.substitutions import TextSubstitution, LaunchConfiguration, Command

def generate_launch_description():

	ld = LaunchDescription()

	parameters_file_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'co_lrio_params.yaml')
	xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'robot.urdf.xacro')

	# Set env var to print message to stdout immediately
	# arg = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')
	# ld.add_action(arg)

	# single robot mapping node
	robot_1_static_node = Node(
		package='tf2_ros',
		namespace='robot_1',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_1/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_1_static_node)

	robot_1_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_1',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_1/'}]
	)
	ld.add_action(robot_1_xacro_node)

	robot_1_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_1',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_1_lidar_odometry_node)

	# robot_1_feature_to_map_matching_node = Node(
	# 	package = 'co_lrio',
	# 	namespace = 'robot_1',
	# 	executable = 'co_lrio_feature_to_map_matching',
	# 	name = 'co_lrio_feature_to_map_matching',
	# 	#parameters = [parameters_file_path],
	# 	arguments = ['--ros-args', '--log-level', 'INFO'],
	# 	output='screen'
	# )
	# ld.add_action(robot_1_feature_to_map_matching_node)

	# single robot mapping node
	robot_0_static_node = Node(
		package='tf2_ros',
		namespace='robot_0',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_0/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_0_static_node)

	robot_0_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_0',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_0/'}]
	)
	ld.add_action(robot_0_xacro_node)

	# robot_0_feature_to_map_matching_node = Node(
	# 	package = 'co_lrio',
	# 	namespace = 'robot_0',
	# 	executable = 'co_lrio_feature_to_map_matching',
	# 	name = 'co_lrio_feature_to_map_matching',
	# 	arguments = ['--ros-args', '--log-level', 'INFO'],
	# 	output='screen'
	# )
	# ld.add_action(robot_0_feature_to_map_matching_node)

	robot_0_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_0',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_0_lidar_odometry_node)

	# single robot mapping node
	robot_2_static_node = Node(
		package='tf2_ros',
		namespace='robot_2',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_2/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_2_static_node)

	robot_2_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_2',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_2/'}]
	)
	ld.add_action(robot_2_xacro_node)

	robot_2_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_2',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_2_lidar_odometry_node)

	# robot_2_feature_to_map_matching_node = Node(
	# 	package = 'co_lrio',
	# 	namespace = 'robot_2',
	# 	executable = 'co_lrio_feature_to_map_matching',
	# 	name = 'co_lrio_feature_to_map_matching',
	# 	arguments = ['--ros-args', '--log-level', 'INFO'],
	# 	output='screen'
	# )
	# ld.add_action(robot_2_feature_to_map_matching_node)

	# centrailized mapping node
	centrailized_mapping_node = Node(
		package = 'co_lrio',
		namespace = '',
		executable = 'co_lrio_concentrated_mapping',
		name = 'co_lrio_concentrated_mapping',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(centrailized_mapping_node)

	# centrailized rviz
	rviz_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_three_robot.rviz')
	rviz_node = Node(
		package = 'rviz2',
		namespace = '',
		executable = 'rviz2',
		name = 'co_lrio_rviz',
		respawn=True, 
		arguments = ['-d' + rviz_config_path]
	)
	ld.add_action(rviz_node)

	# rviza_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_a.rviz')
	# rviza_node = Node(
	# 	package = 'rviz2',
	# 	namespace = '',
	# 	executable = 'rviz2',
	# 	name = 'a_rviz',
	# 	arguments = ['-d' + rviza_config_path]
	# )
	# ld.add_action(rviza_node)

	# rvizb_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_b.rviz')
	# rvizb_node = Node(
	# 	package = 'rviz2',
	# 	namespace = '',
	# 	executable = 'rviz2',
	# 	name = 'b_rviz',
	# 	arguments = ['-d' + rvizb_config_path]
	# )
	# ld.add_action(rvizb_node)

	# rvizc_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_c.rviz')
	# rvizc_node = Node(
	# 	package = 'rviz2',
	# 	namespace = '',
	# 	executable = 'rviz2',
	# 	name = 'c_rviz',
	# 	arguments = ['-d' + rvizc_config_path]
	# )
	# ld.add_action(rvizc_node)

	return ld