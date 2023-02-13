import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#	urdf_file_name = 'elgooso.urdf'
#	urdf = os.path.join(get_package_share_directory('elgooso'), 'urdf',urdf_file_name)
#	with open(urdf, 'r') as infp:
#		robot_desc = infp.read()

	elgooso_path = get_package_share_path('elgooso')
	default_model_path = elgooso_path / 'urdf/elgooso.urdf'
	default_rviz_config_path = elgooso_path / 'rviz/elgooso.rviz'

	model_arg = DeclareLaunchArgument(
		name='model', 
    	default_value=str(default_model_path),
    	description='Absolute path to robot urdf file')
	
	rviz_arg = DeclareLaunchArgument(
		name='rvizconfig', 
		default_value=str(default_rviz_config_path),
		description='Absolute path to rviz config file')
	
	robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

	robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

	#somehow there is a problem with the state_publisher.py file...
	joint_state_publisher_node = Node(
		package='elgooso',
		executable='state_publisher',
		name='state_publisher',
		output='screen'
	)

	rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

	return LaunchDescription([
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use simulation (Gazebo) clock if true'),
		model_arg,
		rviz_arg,
		joint_state_publisher_node,
		robot_state_publisher_node,
		rviz_node
		
#		Node(
#			package='robot_state_publisher',
#			executable='robot_state_publisher',
#			name='robot_state_publisher',
#			output='screen',
#			parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
#			arguments=[urdf]),

])
