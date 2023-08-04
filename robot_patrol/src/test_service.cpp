#include <iostream>
#include <memory>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rmw/qos_profiles.h"
#include "robot_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TestServiceNode : public rclcpp::Node {
public:
  TestServiceNode() : Node("test_service") {
    auto scan_sub_qos =
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    // Subscribe to the laser data
    rclcpp::SubscriptionOptions scan_sub_options;
    scan_sub_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", scan_sub_qos,
        std::bind(&TestServiceNode::processLaserData, this,
                  std::placeholders::_1),
        scan_sub_options);

    // Create the client to call the service
    directionServiceClient_ =
        create_client<robot_interfaces::srv::GetDirection>("direction_service");

    // Wait for the service to be available
    while (
        !directionServiceClient_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }
  }

private:
  void processLaserData(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Create a request object and populate it with the laser scan data
    auto request =
        std::make_shared<robot_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    // Use the service client to call the direction service
    auto future = directionServiceClient_->async_send_request(request);

    // Wait for the response from the service
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      std::string direction = response->direction;

      // Print the received direction
      RCLCPP_INFO(this->get_logger(), "Received direction: %s",
                  direction.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedPtr
      directionServiceClient_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestServiceNode>());
  rclcpp::shutdown();
  return 0;
}