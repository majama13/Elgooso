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
    default_world_path = elgooso_path / 'worlds/world_demo.sdf'
    robot_name = 'elgooso'
    world_name = 'world_demo'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
        parameters=[{'use_sim_time':use_sim_time, 'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    range_sensor_node = Node(
    	package = 'tf2_ros',
    	executable = 'static_transform_publisher',
    	name = 'ultrasonic_sensor',
    	parameters=[{'use_sim_time': use_sim_time}],
    	arguments = ['--x', '0.01', 
    				'--y', '0', 
    				'--z', '0',
    				'--yaw', '0',
    				'--pitch', '0',
    				'--roll', '0',
    				'--frame-id', 'head', 
    				'--child-frame-id', 'elgooso/neck/sonar']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    gazebo = ExecuteProcess(cmd=[['gz sim -v 4 -r ', LaunchConfiguration('world')]], shell=True)
    robot = ExecuteProcess(cmd=[["gz service -s /world/"+world_name+"/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"",LaunchConfiguration('model'),"\", name: \""+ robot_name+"\"'"]], shell=True)
    bridges = Node(
    	package = 'ros_gz_bridge',
    	executable = 'parameter_bridge',
    	arguments = ['/model/elgooso/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    				'/model/elgooso/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    				'/neck_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    				'/sonar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    				'/world/'+world_name+'/remove@ros_gz_interfaces/srv/DeleteEntity',
    				'/world/'+world_name+'/create@ros_gz_interfaces/srv/SpawnEntity'],
    	output='screen'
    )
    
#    twist_bridge = ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /model/elgooso/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']], shell = True)
 #   odom_bridge = ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /model/elgooso/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry']], shell = True)
  #  servo_bridge = ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /neck_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']], shell = True)
   # sensor_bridge = ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /sonar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']], shell = True)
    #delete_bridge = ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /world/'+world_name+'/remove@ros_gz_interfaces/srv/DeleteEntity']], shell = True)
    #spawn_bridge = ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /world/'+world_name+'/create@ros_gz_interfaces/srv/SpawnEntity']], shell = True)
    
    qlearn = Node(
    	package = 'elgooso',
    	executable = 'Qlearn',
    	name = 'Qlearn',
    )


    return LaunchDescription([
        model_arg,
        world_arg,
#        rviz_arg,
#		twist_bridge,
 #       odom_bridge,
  #      servo_bridge,
   #     sensor_bridge,
    #    spawn_bridge,
    	bridges,
        gazebo,
		range_sensor_node,
		robot,
#		robot_state_publisher_node,
#        joint_state_publisher_node,
#        rviz_node,
        qlearn
    ])
