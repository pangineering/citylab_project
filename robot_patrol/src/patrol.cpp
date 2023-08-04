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
    this->declare_parameter("obstacle_threshold", 0.05);
    this->declare_parameter("angular_velocity", 0.2);
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Analyze the laser range data here to determine the best course of action

    // Get the minimum range detected by the laser scan
    float min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    float front_range = msg->ranges[msg->ranges.size() / 2];
    // Read parameters
    float obstacle_threshold =
        this->get_parameter("obstacle_threshold").as_double();
    float angular_velocity =
        this->get_parameter("angular_velocity").as_double();

    // Decide which direction to turn based on the available free space
    // Calculate the range values for the left, front, and right sectors
    size_t num_readings = msg->ranges.size();
    size_t left_index = num_readings / 4;
    size_t right_index = (3 * num_readings) / 4;

    float avg_left_range = 0.0f;
    float avg_front_range = 0.0f;
    float avg_right_range = 0.0f;

    size_t count_left = 0;
    size_t count_front = 0;
    size_t count_right = 0;

    for (size_t i = 0; i < num_readings; ++i) {
      float range = msg->ranges[i];

      if (i < left_index) {
        avg_left_range += range;
        count_left++;
      } else if (i >= left_index && i < right_index) {
        avg_front_range += range;
        count_front++;
      } else {
        avg_right_range += range;
        count_right++;
      }
    }

    if (count_left > 0) {
      avg_left_range /= count_left;
    }

    if (count_front > 0) {
      avg_front_range /= count_front;
    }

    if (count_right > 0) {
      avg_right_range /= count_right;
    }

    // Check if there is an obstacle in any direction
    if (front_range < 0.5) {
      // Stop the robot
      // stop_robot();
      // Choose the direction with the most free space
      if (avg_left_range > avg_right_range) {
        // Turn left
        turn(angular_velocity);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        stop_robot();
      } else {
        // Turn right
        turn(-angular_velocity);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        stop_robot();
      }
    } else {
      // No obstacle, move forward
      move_robot();
    }
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
