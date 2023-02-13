import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

from rclpy.node import Node


class SpawnClient(Node):

    def __init__(self):
        super().__init__('spawn_entity')
        self.client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def send_request(self):
        self.req.name = str(sys.argv[1])
        self.req.xml = str(sys.argv[2])
        self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    spawn = SpawnClient()
    spawn.send_request()

    while rclpy.ok():
        rclpy.spin_once(spawn)
        if spawn.future.done():
            try:
                response = spawn.future.result()
            except Exception as e:
                spawn.get_logger().info(f'Service call failed {e}')
            else:
                spawn.get_logger().info(f'Test {spawn.req.xml}')
            break

    spawn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
