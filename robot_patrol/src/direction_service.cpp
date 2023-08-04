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
  bool directionCallback(
      const robot_interfaces::srv::GetDirection::Request::SharedPtr request,
      const robot_interfaces::srv::GetDirection::Response::SharedPtr response) {
    // Access the laser scan data from the request
    const auto &laserScan = request->laser_data;

    // Divide laser scan into three equal sections
    int numReadings = laserScan.ranges.size();
    int sectionSize = numReadings / 3;
    int section1Start = 0;
    int section1End = sectionSize - 1;
    int section2Start = sectionSize;
    int section2End = 2 * sectionSize - 1;
    int section3Start = 2 * sectionSize;
    int section3End = numReadings - 1;

    // Find the maximum range value from each section
    float maxRange1 = 0.0;
    float maxRange2 = 0.0;
    float maxRange3 = 0.0;

    for (int i = section1Start; i <= section1End; ++i) {
      if (laserScan.ranges[i] > maxRange1) {
        maxRange1 = laserScan.ranges[i];
      }
    }

    for (int i = section2Start; i <= section2End; ++i) {
      if (laserScan.ranges[i] > maxRange2) {
        maxRange2 = laserScan.ranges[i];
      }
    }

    for (int i = section3Start; i <= section3End; ++i) {
      if (laserScan.ranges[i] > maxRange3) {
        maxRange3 = laserScan.ranges[i];
      }
    }

    // Compare the three maximum values to determine the direction
    if (maxRange1 > maxRange2 && maxRange1 > maxRange3) {
      response->direction = "left";
    } else if (maxRange3 > maxRange1 && maxRange3 > maxRange2) {
      response->direction = "right";
    } else {
      response->direction = "forward";
    }

    return true;
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
