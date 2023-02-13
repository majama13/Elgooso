import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import statistics


class Move(Node):

	def __init__(self):
		super().__init__('move')
		self.subscriber_ = self.create_subscription(LaserScan, 'sonar', self.record_scan, 10)
		self.publisher_ = self.create_publisher(Twist, '/model/elgooso/cmd_vel', 10)
		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.distance_ahead = 1000000000

	def record_scan(self, msg):
		self.distance_ahead = statistics.median(msg.ranges)
		self.get_logger().info(f'{self.distance_ahead}')

	def timer_callback(self):
		self.circle()
#		if self.distance_ahead < 2:
#			self.turn_right()
#		else:
#			self.go_straight()

	def go_straight(self):
		t = Twist()
		t.linear.x = 5.0
		self.publisher_.publish(t)

	def turn_right(self):
		t = Twist()
		t.linear.x = 1.0
		t.angular.z = -15.0
		self.publisher_.publish(t)
	
	def circle(self):
		t = Twist()
		t.linear.x = 1.0
		t.angular.z = -15.0
		self.publisher_.publish(t)

def main(args=None):
	rclpy.init(args=args)

	move = Move()

	rclpy.spin(move)

	move.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
