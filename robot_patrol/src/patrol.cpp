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

    // Initialize parameters
    this->declare_parameter("obstacle_threshold", 0.5);
    this->declare_parameter("angular_velocity", 0.2);
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Define the threshold for considering an obstacle
    const float obstacle_threshold = 0.4;
    // Extract distance values for specific angles
    float right_dist = msg->ranges[0];
    float front_dist = msg->ranges[360];
    float left_dist = msg->ranges[90];

    // Print the minimum distances for debugging
    RCLCPP_INFO(get_logger(), "Minimum Right Distance: %f", right_dist);
    RCLCPP_INFO(get_logger(), "Minimum Front Distance: %f", front_dist);
    RCLCPP_INFO(get_logger(), "Minimum Left Distance: %f", left_dist);

    // Define the angular velocity for turning
    float angular_velocity = get_parameter("angular_velocity").as_double();

    // Determine the robot's action based on the distances
    if (front_dist > obstacle_threshold && left_dist > obstacle_threshold &&
        right_dist > obstacle_threshold) {
      // No obstacles, move forward
      move_robot();

    } else if ((front_dist > obstacle_threshold &&
                right_dist < obstacle_threshold &&
                left_dist > obstacle_threshold) ||
               (front_dist > obstacle_threshold &&
                left_dist < obstacle_threshold &&
                right_dist < obstacle_threshold)) {
      // Obstacle on the right, turn left
      stop_robot();
      turn(-0.3);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    } else if ((front_dist < obstacle_threshold &&
                left_dist > obstacle_threshold &&
                right_dist > obstacle_threshold) ||
               (front_dist < obstacle_threshold &&
                left_dist < obstacle_threshold &&
                right_dist > obstacle_threshold)) {
      // Obstacle on the left, turn right
      stop_robot();
      turn(0.3);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    } else if ((front_dist > obstacle_threshold &&
                left_dist < obstacle_threshold &&
                right_dist > obstacle_threshold) ||
               (front_dist < obstacle_threshold &&
                left_dist > obstacle_threshold &&
                right_dist < obstacle_threshold) ||
               (front_dist < obstacle_threshold &&
                left_dist < obstacle_threshold &&
                right_dist < obstacle_threshold)) {
      // Obstacles on both sides or front, check the nearest
      if (left_dist > right_dist) {
        stop_robot();
        turn(0.3);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      } else if (left_dist < right_dist) {
        stop_robot();
        turn(-0.3);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }

    // Sleep to allow the robot to execute the action (adjust the sleep duration
    // as needed)
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Stop the robot after the action
    stop_robot();
  }

  void timer_callback() {
    // Perform useful actions here, e.g., check battery level, update position
    // For demonstration purposes, print a message indicating that the robot
    // is moving
    std::cout << "Robot is moving" << std::endl;
  }

  void move_robot() {
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

  void turn(float angular_velocity) {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = angular_velocity;
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
