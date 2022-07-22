from click import parser
import rclpy
import sys
import argparse
from rclpy.node import Node
from rclpy.action import ActionClient
from hub_interfaces.msg import HubOperation
from hub_interfaces.action import HubAction

class TestHubActionClient(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--company_name', required=True,
                            type=str, help='company name of order')
        parser.add_argument('-o', '--order_id', required=True,
                            type=str, help='order id')
        self.args = parser.parse_args(argv[1:])
        super().__init__('hub_action_client')
        self.action_client = ActionClient(
            self,
            HubAction,
            'hub_action'
        )

    def send_goal(self):
        hub_goal = HubAction.Goal()
        hub_goal.operation.operation = HubOperation.OPERATION_COLLECT
        hub_goal.company_name = self.args.company_name
        hub_goal.order_id = self.args.order_id
        self.action_client.send_goal(hub_goal)


def main():
    rclpy.init()
    action_client = TestHubActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

