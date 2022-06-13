#include <string>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <rclpp/node.hpp>

class Hub
{
public:
  Hub(const std::vector<std::string>& points)
  {
    //constructor
  }

  static std::shared_ptr<Hub> make(const std::vector<std::string>& points);

  /// Collect the item deposited by the robot and store it in one of the
  /// pigeon holes.
  ///
  /// \param[in] company
  ///  Specify the company name of the delivery robot
  ///
  /// \param[in] order_id
  ///
  ///  Specify the order id of the parcel that is to be collected
  bool collect(const std::string& company, const std::string& order_id);

  /// Collect the item deposited by the robot and store it in one of the
  /// pigeon holes.
  ///
  /// \param[in] company
  ///  Specify the company name of the delivery robot
  ///
  /// \param[in] order_id
  ///
  ///  Specify the order id of the parcel that is to be deposited
  bool deposit(const std::string& company, const std::string& order_id);

  /// Check if the collection/ deposit point that the robot is planning to go
  /// to is vaccant
  bool is_open(std::string location)

  /// Get the name of the vacant location that a delivery robot can go to
  /// to collect/ dispense a parcel
  std::string get_vacant();

private:
  std::unordered_map<std::string, std::string> orders = {};

  // Map to store the vacancies of the location
  std::unordered_map<std::string, bool> vaccancy_map = {};



}
