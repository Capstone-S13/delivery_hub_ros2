#include <string>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <uniq
#include <rclcpp/node.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "hub_interfaces/srv/hub_service.hpp"
#include "hub_interfaces/action/hub_move_to.hpp"
#include "hub_interfaces/action/hub_action.hpp"
#include "hub_interfaces/msg/hub_operation.hpp"
#include "hub_interfaces/msg/hub_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "toolhead_interfaces/msg/tool_head_mode.hpp"

using HubAction = hub_interfaces::action::HubAction;
using GoalHandle = rclcpp_action::ServerGoalHandle<HubAction>;

class Hub :public rclcpp::Node
{
public:
  // default constructor
  Hub(const std::vector<std::string>& points);

  /// Factory Pattern for the Hub
  /// \param[in] points
  /// specifies the locations of the pigeon holes

  static std::shared_ptr<Hub> make(const std::vector<std::string>& points);

  bool check_order_exist(const std::string& company,
      const std::string order_id);

  // check if case
  //  collction: order id exists
  //  depositing: a pigeon hole exists
  bool is_valid(
    std::shared_ptr<hub_interfaces::srv::HubService::Response> request);

  void hub_service_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

  /// Get the name of the vacant location that a delivery robot can go to
  /// to collect/ dispense a parcel
  std::string get_vacant();

  struct Order
  {
    std::string company_name;
    std::string order_id;
    bool is_empty = true;
    bool is_reserved = false;
  };


private:
  // this is meant to store the orders into the coresponding pigeon hole number
  std::unordered_map<int, Order> orders;
  uint32_t current_state;
  bool cnc_goal_busy = false;
  bool toolhead_goal_busy = false;
  toolhead_interfaces::msg::ToolHeadMode
  uint32_t desired_next_state;

  std::vector<std::tuple<double, double>> positions;
  // index 0 for collect and 1 for deposits
  std::vector<geometry_msgs::msg::Twist> silder_positions;
  std::vector<geometry_msgs::msg::Twist> deposit_positions;

  // Theses positions should be roughly the positions of
  // deposit_positions but slightly lower.
  std::vector<geometry_msgs::msg::Twist> collect_positions;
  geometry_msgs::msg::Twist hub_position;
  std::mutex hub_position_mutex;
  std::mutex hub_state_mutex;

  // Service server
  rclcpp::Service<hub_interfaces::srv::HubService> hub_service;

  // action server
  rclcpp_action::Server<HubAction> hub_action_server;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist> position_sub;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist> position_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::String> string_cmd_pub;


  // 0 is for collect 1 for deposit
  std::unordered_map<uint32_t, bool> dock_availability_map;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
      const std::shared_ptr<GoalHandle> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

};

