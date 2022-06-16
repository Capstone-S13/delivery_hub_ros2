from argparse import Action

from flask import request
import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.node import Node

from cnc_msgs.action import CncMoveTo
from toolhead_interfaces.action import ToolHeadAction
from hub_interfaces.action import HubAction
from hub_interfaces.srv import HubService
from hub_interfaces.msg import HubOperation, HubState
from hub_python.hub_config import HubConfig
from hub_python.order import Order

class Hub(Node):
    def __init__(self, config):
        super().__init__('hub_node')
        # Hub action server
        self.hub_action_server = ActionServer(
            self,
            HubAction,
            'hub_action',
            self.hub_action_cb
        )

        self.config = config

        self.dock_availability = {HubOperation.OPERATION_COLLECT: True,
            HubOperation.OPERATION_DEPOSIT: True}

        # key = int value = Order
        self.orders = {0: Order(), 1: Order, 2: Order(), 3: Order()}

        self.cnc_goal_busy = False
        self.toolhead_goal_busy = False
        self.state = HubState.IDLE
        self.current_opeartion = None
        self.desired_next_state = None

        # Hub service node
        self.hub_service = self.create_service(
            HubService,
            'hub_service',
            self.hub_service_cb)


        # Tool Head Action client
        self.toolhead_action_client = ActionClient(self, ToolHeadAction, 'toolhead_action')
        self.toolhead_action_client.wait_for_server()

        # Cnc Action client
        self.cnc_action_client = ActionClient(self, CncMoveTo, 'cnc_action')
        self.cnc_action_client.wait_for_server()

        # self.cnc_send_goal_future = None
        # self.toolhead_send_goal_future = None


    #utility functions

    # Checks if an order of a given company and order id exists
    # adds the order if unique
    def check_and_add_order(self, company_name, order_id) -> bool:
        for key in self.orders:
            if(self.orders[key].company_name == company_name
                and self.orders[key].order_id == order_id):
                # Order is not unique
                self.get_logger().info("Order is not unique!")
                return False
        for key in self.orders:
            if(self.orders[key].is_empty == True and
                self.orders[key].is_reserved == False):

                self.orders[key].is_reserved == True
                self.orders[key].company_name == request.company_name
                self.orders[key].order_id = request.order_id
                return True

        self.get_logger().info("Sorry, the hub is full! Maybe try storing \
            it using cloud services?")
        return False

    # cnc action callbacks
    def cnc_goal_response_cb(self, future):
        return

    def cnc_feedback_cb(self, feedback_msg):

        return

    # toolhead action callbacks
    def toolhead_goal_response_cb(self, future):
        return

    def toolhead_feedback_cb(self, feedback_msg):
        return

    # hub service callback
    def hub_service_cb(self, request, response):
        if (request.operation.operation == HubOperation.OPERATION_COLLECT):

            self.get_logger().info(f"Received a Hub request to Collect\n\
                company: {request.company_name}, order_id: {request.order_id}")
            if (self.check_and_add_order(request.company_name, request.order_id)):
                response.is_valid = True
            else:
                response.is_valid = False

            response.available_dock =\
                self.dock_availability[request.operation.operation]
            return response



    def hub_action_cb(self, goal_handle):
        operation_str = ""
        if (goal_handle.request.operation.operation == HubOperation.OPERATION_COLLECT):
            operation_str = "collect"
        elif (goal_handle.request.operation.operation == HubOperation.OPERATION_DEPOSIT):
            operation_str = "deposit"
        self.get_logger().info(f"received hub action:\n\
            operation: {operation_str}\n\
            company_name: {goal_handle.request.company_name}\n\
            order_id: {goal_handle.request.order_id}")

        # first time the robot calls this to get the toolhead into position
        if (goal_handle.request.operation.operation == HubOperation.OPERATION_DEPOSIT\
            and self.state == HubState.IDLE):
            # get toolhead to collect
            cnc_goal = CncMoveTo.Goal()
            cnc_goal.goal.x = self.config.slider_rec_pos[0]
            cnc_goal.goal.y = self.config.slider_rec_pos[2]
            cnc_goal.goal.z = self.config.slider_rec_pos[3]
            self.cnc_send_goal_future =\
                self.cnc_action_client.send_goal_async(cnc_goal, self.cnc_feedback_cb)
            self.cnc_send_goal_future.add_done_callback(self.cnc_goal_response_cb)
            self.state = HubState.MOVING
            self.desired_next_state = HubState.READY_ROBOT_DEPOSIT
            self.current_opeartion = HubOperation.OPERATION_DEPOSIT
            return

        # second time the robot calls this to slide the parcel down
        if (goal_handle.request.operation.operation == HubOperation.OPERATION_DEPOSIT\
            and self.state == HubState.READY_ROBOT_DEPOSIT):
            for key in self.orders:
                if (self.orders[key].company_name == goal_handle.request.company_name
                    and self.orders[key].order_id == goal_handle.request.order_id
                    and self.orders[key].is_empty == True and self.orders[key].is_reserved == True):
                    cnc_goal = CncMoveTo.Goal()
                    pos = self.config.slider_rec_pos

                    # we may want to consider if there is a better place to set this to false
                    # instead as it is not guaranteed that the item will be deposited
                    self.orders[key].is_empty = False
                    cnc_goal.x = pos[0]
                    cnc_goal.y = pos[1]
                    cnc_goal.z = pos[2]
                    self.cnc_send_goal_future =\
                        self.cnc_action_client.send_goal_async(cnc_goal, self.cnc_feedback_cb)
                    self.cnc_send_goal_future.add_done_callback(self.cnc_goal_response_cb)
                    self.state = HubState.MOVING
                    self.desired_next_state = HubState.READY_HUB_COLLECT
                    self.current_opeartion = HubOperation.OPERATION_DEPOSIT
                    return

        if (goal_handle.request.operation.operation == HubOperation.OPERATION_COLLECT
            and self.state == HubState.IDLE):

            for key in self.orders:
                if (self.orders[key].company_name == goal_handle.request.company_name
                    and self.orders[key].order_id == goal_handle.request.order_id
                    and self.orders[key].is_empty == False and self.orders[key].is_reserved == False):
                    pos = self.config.toolhead_collect_pos[key]
                    cnc_goal = CncMoveTo.Goal()
                    cnc_goal.x = pos[0]
                    cnc_goal.y = pos[1]
                    cnc_goal.z = pos[2]
                    self.cnc_send_goal_future =\
                        self.cnc_action_client.send_goal_async(cnc_goal, self.cnc_feedback_cb)
                    self.cnc_send_goal_future.add_done_callback(self.cnc_goal_response_cb)
                    self.state = HubState.MOVING
                    self.desired_next_state = HubState.READY_HUB_DEPOSIT
                    self.current_opeartion = HubOperation.OPERATION_COLLECT
                    return

            return



