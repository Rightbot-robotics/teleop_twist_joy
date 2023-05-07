#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rightbot_interfaces/srv/motor_recovery.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using MotorRecovery = rightbot_interfaces::srv::MotorRecovery;

class MotorRecoveryClient : public rclcpp::Node
{
public:
  MotorRecoveryClient() : Node("motor_recovery_client")
  {
    client_ = create_client<MotorRecovery>("motor_recovery");
  }

  void send_request(const std::string& motor_name, const std::string& function_name)
  {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for the service to appear...");
    }

    auto request = std::make_shared<MotorRecovery::Request>();
    request->motor_name = motor_name;
    request->function_name = function_name;

    auto future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to call motor_recovery service");
      return;
    }

    auto response = future.get();
    RCLCPP_INFO(get_logger(), "Motor recovery response: %d", response->status);
  }

private:
  rclcpp::Client<MotorRecovery>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <motor_name> <function_name>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorRecoveryClient>();
  node->send_request(argv[1], argv[2]);
  rclcpp::shutdown();
  return 0;
}
