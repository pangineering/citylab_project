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
    // Analyze the laser range data here to determine the best course of action

    // Get the minimum range detected by the laser scan
    float min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());

    // Set a threshold distance for obstacle detection
    float obstacle_threshold = 0.5; // 0.2 meters

    // Set a threshold distance for obstacle avoidance
    float avoid_threshold = 0.8; // 0.8 meters

    // Decide which direction to turn based on the available free space
    // Calculate the range values for the left, front, and right sectors
    int num_readings = msg->ranges.size();
    int left_index = num_readings / 4;
    int right_index = 3 * num_readings / 4;
    std::vector<float> left_ranges(msg->ranges.begin(),
                                   msg->ranges.begin() + left_index);
    std::vector<float> front_ranges(msg->ranges.begin() + left_index,
                                    msg->ranges.begin() + right_index);
    std::vector<float> right_ranges(msg->ranges.begin() + right_index,
                                    msg->ranges.end());

    // Calculate the average range value for each sector
    float avg_left_range =
        std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0) /
        left_ranges.size();

    float avg_right_range =
        std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0) /
        right_ranges.size();

    float avg_front_range =
        std::accumulate(front_ranges.begin(), front_ranges.end(), 0.0) /
        front_ranges.size();

    float front_range =
        *std::min_element(front_ranges.begin(), front_ranges.end());

    // Check if there is an obstacle in front of the robot
    if (front_range < obstacle_threshold) {
      // Stop the robot
      stop_robot();

      // Choose the direction with the most free space
      if (avg_left_range > avg_right_range &&
          avg_left_range > avg_front_range && front_range <= 0.4) {
        // Turn left
        turn_left();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
      } else if (avg_right_range > avg_left_range &&
                 avg_right_range > avg_front_range && front_range <= 0.4) {
        // Turn right
        turn_right();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
      } else {
        // No clear direction, move backward
        move_backward();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        // Choose the direction with the most free space again
        if (avg_left_range > avg_right_range) {
          // Turn left
          turn_left();
          rclcpp::sleep_for(std::chrono::milliseconds(1000));
        } else {
          // Turn right
          turn_right();
          rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
      }
    } else {
      // No obstacle, move forward
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
    message.linear.x = 0.1;
    message.angular.z = 0.0;
    publisher_->publish(message);
  }

  void move_backward() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = -0.1;
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
