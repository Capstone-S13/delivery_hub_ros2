import rclpy
import sys
import argparse
from rclpy.node import Node
from hub_interfaces.srv import HubService
from hub_interfaces.msg import HubOperation
import uuid


class TestHubServiceClient(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--company_name', required=True,
                            type=str, help='company name of order')
        parser.add_argument('-o', '--order_id', required=True,
                            type=str, help='order id')
        self.args = parser.parse_args(argv[1:])
        super().__init__('test_hub_service_client')
        self.service_client = self.create_client(HubService, 'hub_service')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for hub service server")

    def send_request(self):
        request = HubService.Request()
        request.operation.operation = HubOperation.OPERATION_COLLECT
        request.company_name = self.args.company_name
        request.order_id = self.args.order_id
        self.future  = self.service_client.call_async(request)

def main():
    rclpy.init()
    client_node = TestHubServiceClient()
    client_node.send_request()
    while rclpy.ok():
        rclpy.spin_once(client_node)
        if client_node.future.done():
            try:
                response = client_node.future.result()
            except Exception as e:
                client_node.get_logger().info(f"service failed: {e}")
        else:
            client_node.get_logger().info("waiting for service")

        break
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



