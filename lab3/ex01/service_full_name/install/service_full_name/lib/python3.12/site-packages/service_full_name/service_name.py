from tutorial_interfaces.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FullNameSumService, 'service_name', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.full_name = request.last_name + request.name + request.first_name
        self.get_logger().info('Incoming request\nfirst_name: %s last_name: %s name: %s' % (request.first_name, request.last_name, request.name))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()