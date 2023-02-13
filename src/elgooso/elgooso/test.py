import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldReset

class Test(Node):
	def __init__(self):
		#put robot back at beginning
		super().__init__('test')
		self.cli = self.create_client(ControlWorld, '~/ros2_ws/src/elgooso/worlds/wall/control')
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.req = ControlWorld.Request()
		self.req.world_control.pause = True
		self.req.world_control.reset.all = True
		
	def send_request(self):
		self.cli.call_async(self.req)

if __name__ == '__main__':
    rclpy.init()
    node = Test()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()
