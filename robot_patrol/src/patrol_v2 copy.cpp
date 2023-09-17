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
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto scan_sub_qos =
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    // Subscribe to the laser data
    rclcpp::SubscriptionOptions scan_sub_options;
    scan_sub_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", scan_sub_qos,
        std::bind(&MoveRobot::laserCallback, this, std::placeholders::_1),
        scan_sub_options);

    directionServiceClient_ =
        this->create_client<robot_interfaces::srv::GetDirection>(
            "/direction_service");
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Analyze the laser range data here to determine the best course of action

    // Existing code ...

    // Create a request object and populate it with the laser scan data
    auto request =
        std::make_shared<robot_interfaces::srv::GetDirection::Request>();
    // Assuming the laser scan data is stored in the `msg` object
    request->laser_data = *msg;

    // Use the service client to call the direction service
    auto future = directionServiceClient_->async_send_request(request);

    // Wait for the response from the service
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      std::string direction = response->direction;

      // Command the robot based on the response direction
      if (direction == "forward") {
        move_robot();
      } else if (direction == "left") {
        turn_left();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        stop_robot();

      } else if (direction == "right") {
        turn_right();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        stop_robot();

        rclcpp::sleep_for(std::chrono::milliseconds(500));
      } else {
        move_backward();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        stop_robot();
      }
    }
  }

  void move_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.4;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }
  void move_backward() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = -0.5;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }
  void slow_down() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.1;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }
  void stop_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  void turn_left() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.2;
    publisher_->publish(message);
  }
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
