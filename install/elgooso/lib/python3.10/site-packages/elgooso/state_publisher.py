from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class JointStatePublisher(Node):

	def __init__(self):
		rclpy.init()
		super().__init__('state_publisher')

		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
		self.nodeName = self.get_name()
		self.get_logger().info(f'{self.nodeName} started')

		degree = pi / 180.0
		loop_rate = self.create_rate(30)

		# robot state
		swivel = 0.
		angle = 0.

		# message declarations
		joint_state = JointState()
		joint_state.header.frame_id = 'world'

		try:
			switch = 1
			while rclpy.ok():
				rclpy.spin_once(self)

				# update joint_state
				now = self.get_clock().now()
				joint_state.header.stamp = now.to_msg()
				joint_state.name = [
					'body_to_neck', 
					'body_to_right_front_wheel',
					'body_to_right_back_wheel',
					'body_to_left_front_wheel',
					'body_to_left_back_wheel']
				joint_state.position = [swivel, angle, angle, angle, angle]

				# send the joint state and transform
				self.joint_pub.publish(joint_state)

				# Create new robot state
				if switch:
					if swivel > 0.70:
						switch = 0
					else:
						swivel += degree
				else:
					if swivel < -0.70:
						switch = 1
					else:
						swivel -= degree

				angle += degree/4

				# This will adjust as needed per iteration
				loop_rate.sleep()

		except KeyboardInterrupt:
			pass

def euler_to_quaternion(roll, pitch, yaw):
	qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
	qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
	qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
	qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
	return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
	node = JointStatePublisher()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
