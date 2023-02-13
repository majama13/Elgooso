from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    elgooso_path = get_package_share_path('elgooso')
    default_model_path = elgooso_path / 'urdf/elgooso.urdf'
    default_rviz_config_path = elgooso_path / 'rviz/elgooso.rviz'
    default_world_path = elgooso_path / 'worlds/wall.world'
    robot_name = 'elgooso'
    world_name = 'wall'

    model_arg = DeclareLaunchArgument(
    	name='model', 
    	default_value=str(default_model_path),
    	description='Absolute path to robot urdf file')
    	
    world_arg = DeclareLaunchArgument(
    	name='world',
    	default_value=str(default_world_path),
    	description='Absolute path to world file')
    	
    rviz_arg = DeclareLaunchArgument(
		name='rvizconfig', 
		default_value=str(default_rviz_config_path),
		description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    gazebo = ExecuteProcess(cmd=[['ign gazebo -v 4 -r ', LaunchConfiguration('world')]], shell=True)
    robot = ExecuteProcess(cmd=[["ign service -s /world/",world_name,"/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"",LaunchConfiguration('model'),"\", name: \"", robot_name,"\"'"]], shell=True)


    return LaunchDescription([
        model_arg,
        world_arg,
        #rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        #rviz_node,
        gazebo,
        robot
    ])
