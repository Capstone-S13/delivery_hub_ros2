import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from hub_interfaces.msg import HubOperation
from hub_interfaces.action import HubAction

class TestHubActionClient(Node):
    def __init__(self):
        super().__init__('hub_action_client')
        self.action_client = ActionClient(
            self,
            HubAction,
            'hub_action'
        )

    def send_goal(self, company_name, order_id):
        hub_goal = HubAction.Goal()
        hub_goal.operation.operation = HubOperation.OPERATION_COLLECT
        hub_goal.company_name = company_name
        hub_goal.order_id = order_id
        self.action_client.send_goal(hub_goal)

def main():
    company_name = "barg"
    order_id = company_name + "my_uid"
    rclpy.init()
    action_client = TestHubActionClient()
    action_client.send_goal(company_name, order_id)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

