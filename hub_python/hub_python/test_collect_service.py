import rclpy
from rclpy.node import Node
from hub_interfaces.srv import HubService
from hub_interfaces.msg import HubOperation
import uuid


class TestHubServiceClient(Node):
    def __init__(self):
        super().__init__('test_hub_service_client')
        self.service_client = self.create_client(HubService, 'hub_service')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for hub service server")

    def send_request(self,company_name, order_id):
        request = HubService.Request()
        request.operation.operation = HubOperation.OPERATION_COLLECT
        request.company_name = company_name
        request.order_id = order_id
        self.future  = self.service_client.call_async(request)

def main():
    company_name = "barg"
    order_id = company_name + "my_uid"
    rclpy.init()
    client_node = TestHubServiceClient()
    client_node.send_request(company_name, order_id)
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



