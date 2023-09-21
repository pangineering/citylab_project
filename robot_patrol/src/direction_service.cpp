#include "robot_interfaces/srv/get_direction.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    // Initialize the service
    directionService_ =
        this->create_service<robot_interfaces::srv::GetDirection>(
            "/direction_service",
            std::bind(&DirectionService::directionCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
  }

  // Callback function for the direction service
  void directionCallback(
      const std::shared_ptr<robot_interfaces::srv::GetDirection::Request>
          request,
      std::shared_ptr<robot_interfaces::srv::GetDirection::Response> response) {
    // Process the request and determine the next action based on the laser data
    // in request->laser_data
    const sensor_msgs::msg::LaserScan &laser_data = request->laser_data;

    // Log the request data for debugging or analysis
    RCLCPP_INFO(this->get_logger(), "Received a request with laser data:");
    RCLCPP_INFO(this->get_logger(), "  Laser Data Header:");
    RCLCPP_INFO(this->get_logger(), "    Frame ID: %s",
                laser_data.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "    Timestamp: %d.%d",
                laser_data.header.stamp.sec, laser_data.header.stamp.nanosec);

    const float front_angle = 0.0;       // Straight ahead
    const float left_angle = M_PI / 2;   // 90 degrees to the left
    const float right_angle = -M_PI / 2; // 90 degrees to the right

    // Calculate the index in the ranges array for each desired angle
    // int front_index = static_cast<int>((front_angle - laser_data.angle_min) /
    //                                   laser_data.angle_increment);
    // int left_index = static_cast<int>((left_angle - laser_data.angle_min) /
    //                                  laser_data.angle_increment);
    // int right_index = static_cast<int>((right_angle - laser_data.angle_min) /
    //                                   laser_data.angle_increment);

    // Ensure the indices are within the valid range
    // front_index =
    //    std::max(0, std::min(front_index,
    //   static_cast<int>(laser_data.ranges.size()) - 1));
    // left_index =
    //    std::max(0, std::min(left_index,
    //  static_cast<int>(laser_data.ranges.size()) - 1));
    // right_index =
    //    std::max(0, std::min(right_index,
    // static_cast<int>(laser_data.ranges.size()) - 1));

    // Extract the distance values for front, left, and right angles
    // float front_dist = laser_data.ranges[front_index];

    // float left_dist = laser_data.ranges[left_index];
    // float right_dist = laser_data.ranges[right_index];
    float right_dist = laser_data.ranges[0];
    float front_dist = laser_data.ranges[360];
    float left_dist = laser_data.ranges[90];

    // Now you have the distances for the specified angles
    RCLCPP_INFO(this->get_logger(), "Front Distance: %.2f meters", front_dist);
    RCLCPP_INFO(this->get_logger(), "Left Distance: %.2f meters", left_dist);
    RCLCPP_INFO(this->get_logger(), "Right Distance: %.2f meters", right_dist);

    // Implement your logic here to determine the next action based on distances
    // For example, if the front distance is less than a threshold, set the
    // response direction to "forward"; otherwise, set it to "stop".
    const float obstacle_threshold = 0.3; // Adjust as needed

    if (front_dist > obstacle_threshold && left_dist > obstacle_threshold &&
        right_dist > obstacle_threshold) {
      // No obstacles, move forward
      response->direction = "forward";
    } else if ((front_dist > obstacle_threshold &&
                right_dist < obstacle_threshold &&
                left_dist > obstacle_threshold) ||
               (front_dist > obstacle_threshold &&
                left_dist < obstacle_threshold &&
                right_dist < obstacle_threshold)) {
      // Obstacle on the right, turn left
      response->direction = "turn_left";
    } else if ((front_dist < obstacle_threshold &&
                left_dist > obstacle_threshold &&
                right_dist > obstacle_threshold) ||
               (front_dist < obstacle_threshold &&
                left_dist < obstacle_threshold &&
                right_dist > obstacle_threshold)) {
      // Obstacle on the left, turn right
      response->direction = "turn_right";
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
        // Obstacle on the right is closer, turn left
        response->direction = "turn_left";
      } else if (left_dist < right_dist) {
        // Obstacle on the left is closer, turn right
        response->direction = "turn_right";
      }
    } else {
      // Default action (if none of the conditions are met)
      response->direction = "stop";
    }
  }

private:
  rclcpp::Service<robot_interfaces::srv::GetDirection>::SharedPtr
      directionService_; // Service server for the direction service
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
