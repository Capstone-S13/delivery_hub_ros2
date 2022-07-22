#include "hub/hub.hpp"


Hub::Hub(const std::vector<std::string>& points)
{
    //constructor
  using namespace std::placeholders;
  // create action server
  this->hub_action_server_ = rclcpp_action::create_server<HubAction>(
      this,
      "hub",
      std::bind(&Hub::handle_goal, this, _1, _2),
      std::bind(&Hub::handle_cancel, this, _1),
      std::bind(&Hub::handle_accepted, this, _1));

  // create service

  this->create_subscriber('/cnc_interface/position', 10,
  std::bind(&Hub::hub_position_cb, this, std::placeholders::_1));
  this->create_publisher('cnc_interface/pos', 10);
  this->create_publisher('cnc_interface/cmd', 10);
}

static std::shared_ptr<Hub> Hub::make(std::vector<std::string>)
{
  std::vector<std::string> orders;
  auto hub_ptr = std::make_shared<Hub>(list);

}

// Misc Callback Methods
void Hub::hub_position_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock(hub_position_mutex);
  hub_position.linear.x= msg->linear.x;
  hub_position.linear.y= msg->linear.y;
  hub_position.linear.z= msg->linear.z;
}


// HubService Methods
void Hub::hub_service_cb(
    const std::shared_ptr<hub_interfaces::srv::HubService::Request> request,
    const std::shared_ptr<hub_interfaces::srv::HubService::Response> response
)
{
  response->available_dock = dock_availability_map[request->operation];

  bool is_valid;
  if (request->operation == hub_interfaces::msg::HubOperation::OPERATION_DEPOSIT)
  {
    is_valid = check_deposit(request->company_name, request->order_id);
    response->is_valid = is_valid;
    return;
  }

  else if (request->operation ==
    hub_interfaces::msg::HubOperation::OPERATION_COLLECT)
  {
    is_valid = check_collect(request->company_name, request->order_id);
    response->is_valid = is_valid;
  }

}

bool Hub::service_deposit(const std::string _company_name,
    const std::string& _order_id)
{
  for (auto& it : vacancy_map)
  {
    // if all are reserved
    if (!it.second.is_reserved)
    {
      it.second.is_reserved = true;
      it.second.company_name = request->company_name;
      it.second.order_id = request->order_id;
      return true;
    }
  }
  return false;
}

bool Hub::service_collect(const std::string& _company_name,
    const std::string& _order_id)
{
  for (auto& it : vacancy_map)
  {
    if (it.second.company_name == _company_name &&
      it.second.order_id == _order_id && !it.second.is_empty && !it.second.is_reserved)
      return true;
  }
  return false;
}

rclcpp_action::GoalResponse Hub::handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<GoalHandle> goal_handle)
{
  std::string operation;
  if (goal_handle->operation ==
      hub_interfaces::msg::HubOperation::OPERATION_DEPOSIT)
    operation = "deposit";
  else if (goal_handle->operation ==
      hub_interfaces::msg::HubOperation::OPERATION_COLLECT)
      operation == "collect";

  RCLCPP_INFO(this->get_logger(), "Received action request with operation:
      %d and company: %s and order id: %s", operation,
      goal_handle->company_name, goal_handle->order_id);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Hub::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void) goal_handle;
  return rclcpp_action::CancelRespose::ACCEPT;
}

void Hub::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Hub::execute, this, _1), goal_handle}.detach();
}

void Hub::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<HubAction::Feedback>();
  auto result = std::make_shared<HubAction::Result>();


  // hub state should not be idle at any time when it is trying to  comlet
  // an operation

  // move toolhead to position to collect from robot

  std::unique_lock<std::mutex> hub_state_lock(hub_state_mutex);
  if (goal->operation.operation ==
      hub_interfaces::msg::HubOperation::OPERATION_DEPOSIT &&
      this->state == hub_interfaces::msg::HubState::IDLE)
  {
    hub_state_lock.unlock();
    this->position_cmd_pub.publish(this->slider_positions[1]);
    bool in_pos = false;
    rclcpp::Rate loop_rate(0.5);
    while(!in_pos)
    {
      {
        std::unique_lock<std::mutex> lock(hub_position_mutex);
        if (this->hub_position.linear.x == this->slider_position[1].linear.x &&
            this->hub_position.linear.y == this->slider_position[1].linear.y &&
            this->hub_position.linear.z == this->slider_position[1].linear.z &&)
          in_pos = true;
      }
      feed_back->state = hub_interfaces::msg::HubState::MOVING;
      goal_handle->publish(feedback);
      loop_rate.sleep();
    }
    //spin wheels here
    this->state == hub_interfaces::msg::HubState::READY_ROBOT_DEPOSIT;
    feed_back->state = hub_interfaces::msg::HubState::READY_ROBOT_DEPOSIT;
    if (rclcpp::ok())
    {
      result->in_pos = true;
      goal_handle->publish(result);
      return;
    }
  }

  // move toolhead to position to pigeon hole to deposit
  std::unique_lock<std::mutex> hub_state_lock(hub_state_mutex);
  if (goal->operation ==
      hub_interfaces::msg::HubOperation::OPERATION_DEPOSIT &&
      this->state == hub_interfaces::msg::HubState::READY_ROBOT_DEPOSIT)
  {
    // get postion of pigeon hole
    int pigeon_hole_index;
    for (auto it& : this->vacancy_map)
    {
      if (it.second.company_name == goal->company_name &&
          it.second.order_id == goal->order_id &&
          it.second.is_empty)
        {
          it.second.is_empty == false;
          it.second.is_reserved == false;
          pigeon_hole_index = it.first;
          break;
        }
    }
    geometry_msgs::msg::Twist target_pos =
        this->deposit_positions[pigeon_hole_index];
    this->position_cmd_pub.publish(target_pos);
    bool in_pos = false;
    rclcpp::Rate loop_rate(0.5);
    while(!in_pos)
    {
      {
        std::unique_lock<std::mutex> lock(hub_position_mutex);
        if (this->hub_position.linear.x == target_pos.linear.x &&
            this->hub_position.linear.y == target_pos.linear.y &&
            this->hub_position.linear.z == target_pos.linear.z &&)
          in_pos = true;
      }
      feed_back->state = hub_interfaces::msg::HubState::MOVING;
      this->state == hub_interfaces::msg::HubState::MOVING;
      goal_handle->publish(feedback);
      loop_rate.sleep();
    }
    feed_back->state = hub_interfaces::msg::HubState::READY_HUB_DEPOSIT;
    this->state == hub_interfaces::msg::HubState::READY_HUB_DEPOSIT;
    goal_handle->publish(feedback);
    // call action to tilt the hub
    if (rclcpp::ok())
    {
      feed_back->state = hub_interfaces::msg::HubState::IDLE;
      this->state == hub_interfaces::msg::HubState::IDLE;
      result->in_pos = true;
      goal_handle->publish(result);
      RCLCPP_INFO(this->get_logger(), "Das paket ist in das Fach!");
      return;
    }
  }

// move toolhead to position to pigeon hole to collect
// unlike depositing we can execute this whole operation with just action call
  { //set the scope of the hub_state lock
    std::unique_lock<std::mutex> lock(hub_state_mutex);
    if (goal->operation ==
        hub_interfaces::msg::HubOperation::OPERATION_COLLECT &&
        this->state == hub_interfaces::msg::HubState::IDLE)
    {

      // get postion of pigeon hole
      int pigeon_hole_index;
      for (auto it& : this->vacancy_map)
      {
        if (it.second.company_name == goal->company_name &&
            it.second.order_id == goal->order_id &&
            it.second.is_empty)
          {
            it.second.is_empty == false;
            it.second.is_reserved == false;
            pigeon_hole_index = it.first;
            break;
          }
      }
      geometry_msgs::msg::Twist target_pos =
          this->deposit_positions[pigeon_hole_index];
      this->position_cmd_pub.publish(target_pos);
      this->state == hub_interfaces::msg::HubState::READY_ROBOT_MOVING;
      bool in_pos = false;
      rclcpp::Rate loop_rate(0.5);

      while(!in_pos)
      {
        {
          std::unique_lock<std::mutex> lock(hub_position_mutex);
          if (this->hub_position.linear.x == target_pos.linear.x &&
              this->hub_position.linear.y == target_pos.linear.y &&
              this->hub_position.linear.z == target_pos.linear.z &&)
            in_pos = true;
        }
        feed_back->state = hub_interfaces::msg::HubState::MOVING;
        goal_handle->publish(feedback);
        loop_rate.sleep();
      }
      feed_back->state = hub_interfaces::msg::HubState::READY_HUB_DEPOSIT;
      goal_handle->publish(feedback);
      // call action to tilt the hub

      if (rclcpp::ok())
      {
        result->in_pos = true;
        goal_handle->publish(result);
        RCLCPP_INFO(this->get_logger(), "Das paket ist in das Fach!");
        return;
      }
    }
  }


}




