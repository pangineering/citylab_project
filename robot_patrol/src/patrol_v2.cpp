#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/get_direction.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("robot_patrol2") {
    // Create a publisher to publish Twist messages to the cmd_vel topic
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create a subscription to the laser scan data
    auto scan_sub_qos =
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    rclcpp::SubscriptionOptions scan_sub_options;
    scan_sub_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", scan_sub_qos,
        std::bind(&MoveRobot::processLaserData, this, std::placeholders::_1),
        scan_sub_options);

    // Create a client to call the direction service
    directionServiceClient_ =
        this->create_client<robot_interfaces::srv::GetDirection>(
            "/direction_service");
  }

private:
  void processLaserData(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Create a request object and populate it with the laser scan data
    auto request =
        std::make_shared<robot_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    // Use the service client to call the direction service with a callback
    directionServiceClient_->async_send_request(
        request, std::bind(&MoveRobot::handleServiceResponse, this,
                           std::placeholders::_1));
  }

  void handleServiceResponse(
      rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedFuture
          future) {
    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto response = future.get();
      std::string direction = response->direction;

      // Print the received direction
      RCLCPP_INFO(this->get_logger(), "Received direction: %s",
                  direction.c_str());
      std::string direction_1 = direction.c_str();
      // Take action based on the received direction
      if (direction_1 == "forward") {
        move_robot();
      } else if (direction_1 == "turn_left") {
        stop_robot();
        turn_left();
        rclcpp::sleep_for(std::chrono::milliseconds(100));

      } else if (direction_1 == "turn_right") {
        stop_robot();
        turn_right();
        rclcpp::sleep_for(std::chrono::milliseconds(100));

      } else if (direction_1 == "stop") {
        stop_robot();
      } else {
        move_backward();
      }

      // Log that the service was called
      RCLCPP_INFO(this->get_logger(), "Service was called.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
    }
  }

  // Move the robot forward
  void move_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  // Move the robot backward
  void move_backward() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = -0.2;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  // Slow down the robot
  void slow_down() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.1;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  // Stop the robot
  void stop_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  // Turn the robot left
  void turn_left() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.2;
    publisher_->publish(message);
  }

  // Turn the robot right
  void turn_right() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = -0.2;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedPtr
      directionServiceClient_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto move_robot = std::make_shared<MoveRobot>();
  rclcpp::spin(move_robot);
  rclcpp::shutdown();
  return 0;
}