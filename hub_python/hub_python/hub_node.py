from difflib import restore
import time

from matplotlib.pyplot import rc
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
from toolhead_interfaces.msg import ToolHeadMode
from action_msgs.msg import GoalStatus

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
        self.orders= {}

        self.cnc_goal_busy = False
        self.toolhead_goal_busy = False
        self.state = HubState.IDLE
        self.current_operation = None
        self.desired_next_state = None

        self.wheel_state = None
        self.pusher_state = None
        self.wheel_next_state = None
        self.pusher_next_state = None

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
        self.get_logger().info("hub node is up!")
        self.init_order_map()


    #utility functions
    def init_order_map(self):
        # check there are coordinates for collecting and depositing for each pigeon hole
        assert(len(self.config.toolhead_collect_pos) == len(self.config.toolhead_deposit_pos))
        # check if all values for coordinates are floats
        for i in range(len(self.config.toolhead_collect_pos)):
            for j in range(len(self.config.toolhead_collect_pos[i])):
                assert(type(self.config.toolhead_collect_pos[i][j]) == float)
        for i in range(len(self.config.toolhead_deposit_pos)):
            for j in range(len(self.config.toolhead_deposit_pos[i])):
                assert(type(self.config.toolhead_deposit_pos[i][j]) == float)

        # set up the order map
        for i in range(len(self.config.toolhead_collect_pos)):
            self.orders[i] = Order()

        return

    # Checks if an order of a given company and order id exists
    # adds the order if unique
    def check_order_exist(self, company_name, order_id) -> bool:
        for key in self.orders:
            if (self.orders[key].company_name == company_name
                and self.orders[key].order_id == order_id):
                return True
        return False

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

                self.orders[key].is_reserved = True
                self.orders[key].company_name = company_name
                self.orders[key].order_id = order_id
                self.get_logger().info(f"successfully added order from\
                    {company_name} of order id: {order_id}")
                return True

        self.get_logger().info("Sorry, the hub is full! Maybe try storing \
            it using cloud services?")
        return False

    def unreserve_order(self, company_name, order_id):
        for key in self.orders:
            if (self.orders[key].company_name == company_name
                and self.orders[key].order_id == order_id):
                self.orders[key].is_reserved = False

    def reset_order(self, company_name, order_id):
        for key in self.orders:
            if (self.orders[key].company_name == company_name
                and self.orders[key].order_id == order_id):
                self.orders[key] = Order()
                return True
        return False

    def get_cnc_goal_from_order(self, company_name, order_id):
        cnc_goal = None
        if (self.current_operation == HubOperation.OPERATION_COLLECT):
            for key in self.orders:
                if (self.orders[key].company_name == company_name
                    and self.orders[key].order_id == order_id
                    and self.orders[key].is_empty == False
                    and self.orders[key].is_reserved == False):
                    self.orders[key].is_reserved = True
                    pos = self.config.toolhead_collect_pos[key]
                    cnc_goal = CncMoveTo.Goal()
                    cnc_goal.goal.x = pos[0]
                    cnc_goal.goal.y = pos[1]
                    cnc_goal.goal.z = pos[2]
        elif(self.current_operation == HubOperation.OPERATION_DEPOSIT):
            for key in self.orders:
                if (self.orders[key].company_name == company_name
                    and self.orders[key].order_id == order_id
                    and self.orders[key].is_empty == True
                    and self.orders[key].is_reserved == True):
                        pos = self.config.toolhead_deposit_pos[key]
                        self.orders[key].is_empty = False
                        cnc_goal = CncMoveTo.Goal()
                        cnc_goal.goal.x = pos[0]
                        cnc_goal.goal.y = pos[1]
                        cnc_goal.goal.z = pos[2]
        return cnc_goal


    def get_slider_rec_pos(self):
        cnc_goal = CncMoveTo.Goal()
        cnc_goal.goal.x = self.config.slider_rec_pos[0]
        cnc_goal.goal.y = self.config.slider_rec_pos[1]
        cnc_goal.goal.z = self.config.slider_rec_pos[2]
        return cnc_goal

    def get_slider_send_pos(self):
        cnc_goal = CncMoveTo.Goal()
        cnc_goal.goal.x = self.config.slider_send_pos[0]
        cnc_goal.goal.y = self.config.slider_send_pos[1]
        cnc_goal.goal.z = self.config.slider_send_pos[2]
        return cnc_goal

    # executing goals
    def execute_cnc_goal(self, cnc_goal, desired_next_state):
        self.desired_next_state = desired_next_state
        self.cnc_send_goal_future =\
            self.cnc_action_client.send_goal_async(
                cnc_goal,
                feedback_callback=self.cnc_feedback_cb
            )
        self.state = HubState.MOVING
        self.cnc_send_goal_future.add_done_callback(self.cnc_goal_response_cb)
        return

    def execute_toolhead_goal(self, pusher_next_state, wheel_next_state):
        # create goal
        toolhead_goal = ToolHeadAction.Goal()
        self.pusher_next_state = pusher_next_state
        self.wheel_next_state = wheel_next_state
        self.pusher_state = ToolHeadMode.PUSHER_MOVING
        self.wheel_state = wheel_next_state
        toolhead_goal.pusher_action.operation = self.pusher_next_state
        toolhead_goal.wheel_action.operation = self.wheel_next_state

        self.get_logger().info("sending toolhead")
        self.toolhead_send_goal_future =\
            self.toolhead_action_client.send_goal_async(
                toolhead_goal,\
                feedback_callback=self.toolhead_feedback_cb
            )
        self.toolhead_send_goal_future.\
            add_done_callback(self.toolhead_goal_response_cb)
        return

    # busy wait functions for waiting for goals to complete
    def wait_for_cnc(self):
        while (self.state != self.desired_next_state):
            self.get_logger().info("waiting for cnc")
            rclpy.spin_once(self)
        self.get_logger().info("cnc done")
        return

    def wait_for_toolhead(self):
        while (self.wheel_state != self.wheel_next_state and
            self.pusher_state != self.pusher_next_state):
            self.get_logger().info("waiting for toolhead")
            rclpy.spin_once(self)
        self.get_logger().info("toolhead done")
        return


    # call back when the goal is accepted/rejected
    def cnc_goal_response_cb(self, future):
        goal_handle = future.result()
        if (not goal_handle.accepted):
            self.get_logger().info("cnc goal rejected")
            return
        self.cnc_get_result_future = goal_handle.get_result_async()
        self.cnc_get_result_future.add_done_callback(self.cnc_get_result_cb)
        return

    # callback when the goal is done
    def cnc_get_result_cb(self, future):
        result = future.result().result
        self.state = self.desired_next_state
        self.get_logger().info("cnc goal reached")
        return

    # feedback callback
    def cnc_feedback_cb(self, feedback_msg):
        self.get_logger().info(f"cnc at position x: {feedback_msg.feedback.position.x},\
            y: {feedback_msg.feedback.position.y}, z: {feedback_msg.feedback.position.z}")
        return

    # toolhead action callbacks
    def toolhead_goal_response_cb(self, future):
        goal_handle = future.result()
        if (not goal_handle.accepted):
            self.get_logger().info("toolhead goal rejected")
        self.get_logger().info("toolhead goal accepted")
        self.toolhead_get_result_future = goal_handle.get_result_async()
        self.toolhead_get_result_future.add_done_callback(self.toolhead_get_result_cb)
        return

    def toolhead_get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info("toolhead goal reached")
        self.wheel_state = self.wheel_next_state
        self.pusher_state = self.pusher_next_state
        return

    def toolhead_feedback_cb(self, feedback_msg):
        self.get_logger().info(f"executing toolhead goal")
        return

    # hub service callback
    def hub_service_cb(self, request, response):
        response.available_dock =\
                self.dock_availability[request.operation.operation]
        response.is_valid = False
        if (request.operation.operation == HubOperation.OPERATION_DEPOSIT):

            self.get_logger().info(f"Received a Hub request to deposit\n\
                company: {request.company_name}, order_id: {request.order_id}")
            if (self.check_and_add_order(request.company_name, request.order_id)):
                self.get_logger().info("adding order")
                response.is_valid = True
                self.dock_availability[request.operation.operation] = False
                return response
            self.get_logger().info("order isn't added as it isnt valid!")
            return response

        elif (request.operation.operation == HubOperation.OPERATION_COLLECT):
            if self.check_order_exist(request.company_name, request.order_id):
                self.get_logger().info("Yahoo! order exists!")
                response.is_valid = True
                self.dock_availability[request.operation.operation] = False
                return response
            self.get_logger.info("order is not valid!")
            return response
        return response


    # hub action callback
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

        hub_action_result  = HubAction.Result()
        # first time the robot calls this to get the toolhead into position
        if (goal_handle.request.operation.operation == HubOperation.OPERATION_DEPOSIT\
            and self.state == HubState.IDLE):
            # get toolhead to collect
            self.current_operation = HubOperation.OPERATION_DEPOSIT
            cnc_goal = self.get_slider_rec_pos()
            self.execute_cnc_goal(cnc_goal, HubState.READY_ROBOT_DEPOSIT)
            self.wait_for_cnc()

            # spin in to receieve parcel sliding down from robot
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN, ToolHeadMode.SPIN_IN)
            self.wait_for_toolhead()

            self.desired_next_state = HubState.READY_HUB_COLLECT
            goal_handle.succeed()
            hub_action_result.state.state = self.state
            hub_action_result.in_position = True
            return hub_action_result

        # second time the robot calls this to slide the parcel down
        if (goal_handle.request.operation.operation == HubOperation.OPERATION_DEPOSIT\
            and self.state == HubState.READY_ROBOT_DEPOSIT):

            # stop wheel for spinning
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN, ToolHeadMode.SPIN_IDLE)
            self.wait_for_toolhead()

            # move toolhead to the pigeon hole to deposit parcel
            cnc_goal = self.get_cnc_goal_from_order(goal_handle.request.company_name,\
                goal_handle.request.order_id)
            if (cnc_goal == None):
                self.get_logger().info("no such order.. die")
            self.execute_cnc_goal(cnc_goal, HubState.READY_HUB_COLLECT)
            self.wait_for_cnc()

            # spin wheel out to move parcel in
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN,ToolHeadMode.SPIN_OUT)
            self.wait_for_toolhead()
            time.sleep(3)

            # stop spinning wheels
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN, ToolHeadMode.SPIN_IDLE)
            self.wait_for_toolhead()

            hub_action_result.state.state = self.state
            hub_action_result.in_position = True
            self.current_operation = None
            self.unreserve_order(goal_handle.request.company_name,\
                goal_handle.request.order_id)
            self.state = HubState.IDLE
            self.get_logger().info("operation deposit complete!")
            return hub_action_result

        # collection is a single process
        if (goal_handle.request.operation.operation == HubOperation.OPERATION_COLLECT
            and self.state == HubState.IDLE):
            self.get_logger().info("commencing collect operation")
            self.current_operation = HubOperation.OPERATION_COLLECT

            # get the order
            cnc_goal = self.get_cnc_goal_from_order(
                goal_handle.request.company_name,
                goal_handle.request.order_id)

            if (cnc_goal == None):
                self.get_logger().info("Hong gan liao. No such order!")
                hub_action_result.state.state = self.state
                hub_action_result.in_position = False
                return hub_action_result

            # set the cnc to be in position toolhead to push and collect
            self.execute_cnc_goal(cnc_goal, HubState.READY_HUB_DEPOSIT)
            self.wait_for_cnc()

            # toolhead operations for toolhead to receive parcel
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_OUT, ToolHeadMode.SPIN_IN)
            self.wait_for_toolhead()
            time.sleep(3)

            # stop spinning an retract toolhead pusher to move cnc again
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN, ToolHeadMode.SPIN_IDLE)
            self.wait_for_toolhead()

            # give some leeway for the the pusher to be retracted back
            time.sleep(3)

            # move the cnc so toolhead is in position to slide parcel to robot
            cnc_goal = self.get_slider_send_pos()
            self.execute_cnc_goal(cnc_goal, HubState.READY_ROBOT_COLLECT)
            self.wait_for_cnc()

            # spin wheel to slide parcel down
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN, ToolHeadMode.SPIN_OUT)
            self.wait_for_toolhead()
            time.sleep(3)

            # stop spinning
            self.execute_toolhead_goal(ToolHeadMode.PUSHER_IN,ToolHeadMode.SPIN_IDLE)
            self.wait_for_toolhead()

            hub_action_result.state.state = self.state
            hub_action_result.in_position = True
            self.state = HubState.IDLE
            self.current_operation = None
            self.reset_order(goal_handle.request.company_name,\
                goal_handle.request.order_id)
            return hub_action_result


def main():
    rclpy.init()
    hub_config = HubConfig()
    hub = Hub(hub_config)
    while(rclpy.ok()):
        rclpy.spin_once(hub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
