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

	parameters_file_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'co_lrio_sim_params.yaml')
	xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'robot.urdf.xacro')

	# Set env var to print message to stdout immediately
	# arg = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')
	# ld.add_action(arg)

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

	# single robot mapping node
	robot_3_static_node = Node(
		package='tf2_ros',
		namespace='robot_3',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_3/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_3_static_node)

	robot_3_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_3',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_3/'}]
	)
	ld.add_action(robot_3_xacro_node)

	robot_3_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_3',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_3_lidar_odometry_node)

	# single robot mapping node
	robot_4_static_node = Node(
		package='tf2_ros',
		namespace='robot_4',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_4/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_4_static_node)

	robot_4_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_4',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_4/'}]
	)
	ld.add_action(robot_4_xacro_node)

	robot_4_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_4',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_4_lidar_odometry_node)

	# single robot mapping node
	robot_5_static_node = Node(
		package='tf2_ros',
		namespace='robot_5',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_5/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_5_static_node)

	robot_5_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_5',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_5/'}]
	)
	ld.add_action(robot_5_xacro_node)

	robot_5_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_5',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_5_lidar_odometry_node)

	# single robot mapping node
	robot_6_static_node = Node(
		package='tf2_ros',
		namespace='robot_6',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_6/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_6_static_node)

	robot_6_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_6',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_6/'}]
	)
	ld.add_action(robot_6_xacro_node)

	robot_6_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_6',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_6_lidar_odometry_node)

	# single robot mapping node
	robot_7_static_node = Node(
		package='tf2_ros',
		namespace='robot_7',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_7/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_7_static_node)

	robot_7_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_7',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_7/'}]
	)
	ld.add_action(robot_7_xacro_node)

	robot_7_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_7',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_7_lidar_odometry_node)

	# single robot mapping node
	robot_8_static_node = Node(
		package='tf2_ros',
		namespace='robot_8',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_8/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_8_static_node)

	robot_8_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_8',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_8/'}]
	)
	ld.add_action(robot_8_xacro_node)

	robot_8_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_8',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_8_lidar_odometry_node)

	# single robot mapping node
	robot_9_static_node = Node(
		package='tf2_ros',
		namespace='robot_9',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_9/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_9_static_node)

	robot_9_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_9',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_9/'}]
	)
	ld.add_action(robot_9_xacro_node)

	robot_9_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_9',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_9_lidar_odometry_node)

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

	# gt node
	# gt_node = Node(
	# 	package = 'co_lrio',
	# 	namespace = '',
	# 	executable = 'co_lrio_gt_node',
	# 	name = 'co_lrio_gt_node',
	# 	parameters = [parameters_file_path],
	# 	arguments = ['--ros-args', '--log-level', 'INFO'],
	# 	output='screen'
	# )
	# ld.add_action(gt_node)

	# centrailized rviz
	rviz_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2.rviz')
	rviz_node = Node(
		package = 'rviz2',
		namespace = '',
		executable = 'rviz2',
		name = 'co_lrio_rviz',
		arguments = ['-d' + rviz_config_path]
	)
	ld.add_action(rviz_node)

	return ld