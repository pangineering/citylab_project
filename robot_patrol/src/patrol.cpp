#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("robot_patrol") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10),
        std::bind(&MoveRobot::laserCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MoveRobot::timer_callback, this));
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Extract the range data from the front laser
    auto front_ranges =
        std::vector<float>(msg->ranges.begin() + (msg->ranges.size() / 2),
                           msg->ranges.begin() + (msg->ranges.size() / 2) + 1);

    // Analyze the laser range data here
    // For simplicity, we will implement a basic obstacle avoidance behavior
    // based on the range detected by the front laser

    // Get the range detected by the front laser
    float front_range = front_ranges[0];

    // Set a threshold distance for obstacle detection
    float obstacle_threshold = 0.4; // 0.2 meters

    // Check if there is an obstacle in front of the robot
    if (front_range < obstacle_threshold) {
      // Stop the robot
      stop_robot();

      // Turn around to avoid getting stuck
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      move_backward();
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      turn_left();
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      stop_robot(); // Stop after turning
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      move_robot(); // Move forward after turning
    } else {
      // Move the robot forward
      move_robot();
    }
  }

  void timer_callback() {
    // For demonstration purposes, print a message indicating that the robot
    // is moving
    std::cout << "Robot is moving" << std::endl;
  }

  void move_robot() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }
  void move_backward() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = -0.5;
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto move_robot = std::make_shared<MoveRobot>();
  rclcpp::spin(move_robot);
  rclcpp::shutdown();
  return 0;
}
