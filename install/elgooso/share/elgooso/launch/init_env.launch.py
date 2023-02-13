from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    elgooso_path = get_package_share_path('elgooso')
    default_model_path = elgooso_path / 'urdf/elgooso.urdf'
#    default_world_path = elgooso_path / 'worlds/world_demo.sdf'
    robot_name = 'elgooso'
    world_name = 'world_demo'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    model_arg = DeclareLaunchArgument(
    	name='model', 
    	default_value=str(default_model_path),
    	description='Absolute path to robot urdf file')
    	
#    world_arg = DeclareLaunchArgument(
#    	name='world',
#    	default_value=str(default_world_path),
#    	description='Absolute path to world file')

#    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
#    range_sensor_node = Node(
#    	package = 'tf2_ros',
#    	executable = 'static_transform_publisher',
#    	name = 'ultrasonic_sensor',
#    	parameters=[{'use_sim_time': use_sim_time}],
#    	arguments = ['--x', '0.01', 
#    				'--y', '0', 
#    				'--z', '0',
#    				'--yaw', '0',
#    				'--pitch', '0',
#    				'--roll', '0',
#    				'--frame-id', 'head', 
#    				'--child-frame-id', 'elgooso/neck/sonar']
#    )
    
#    ign service -s /world/empty/create \
#		--reqtype ignition.msgs.EntityFactory \
#		--reptype ignition.msgs.Boolean \
#		--timeout 300 \
#		--req 'sdf: '\
#		'"<?xml version=\"1.0\" ?>'\
#		'<sdf version=\"1.6\">'\
#		'<model name=\"spawned_model\">'\
#		'<link name=\"link\">'\
#		'<visual name=\"visual\">'\
#		'<geometry><sphere><radius>1.0</radius></sphere></geometry>'\
#		'</visual>'\
#		'<collision name=\"visual\">'\
#		'<geometry><sphere><radius>1.0</radius></sphere></geometry>'\
#		'</collision>'\
#		'</link>'\
#		'</model>'\
#		'</sdf>" '\
#		'pose: {position: {z: 10}} '\
#		'name: "new_name" '\
#		'allow_renaming: true'
    
    robot = ExecuteProcess(cmd=[["ign service -s /world/",world_name,"/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"",LaunchConfiguration('model'),"\", name: \"", robot_name,"\"'"]], shell=True)


    return LaunchDescription([
        model_arg,
        world_arg,
		range_sensor_node,
		robot,
    ])
