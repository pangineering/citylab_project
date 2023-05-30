#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/action/go_to_point.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class GoToPoint : public rclcpp::Node {
public:
  explicit GoToPoint()
      : Node("go_to_point_action_node"), goal_received_(false) {
    // Create an action server with the name "/go_to_point"
    action_server_ = rclcpp_action::create_server<robot_interfaces::action::GoToPoint>(
        this, "/go_to_point",
        std::bind(&GoToPoint::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GoToPoint::handle_cancel, this, std::placeholders::_1),
        std::bind(&GoToPoint::handle_accepted, this, std::placeholders::_1));
  }

private:
  // Action goal callback
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const robot_interfaces::action::GoToPoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    // Perform any necessary checks/validation on the goal
    // For example, check if the goal is valid or achievable
    // Return ACCEPT if the goal is accepted, or REJECT if it's rejected
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Action cancel callback
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::GoToPoint>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // Perform any necessary cancellation operations
    // Return CANCELLED to indicate the goal has been successfully canceled
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Action accepted callback
  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::GoToPoint>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    // Start processing the goal
    goal_received_ = true;
    execute_goal(goal_handle);
  }

  // Execute the action goal
  void execute_goal(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::GoToPoint>> goal_handle) {
    // Get the feedback and result publishers for the action
    auto feedback = std::make_shared<robot_interfaces::action::GoToPoint::Feedback>();
    auto result = std::make_shared<robot_interfaces::action::GoToPoint::Result>();

    // Initialize the feedback and result
    feedback->progress = 0.0;
    result->success = false;

    // Create a publisher to control the robot's velocity
    auto velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a timer to control the control loop
    auto timer = this->create_wall_timer(100ms, [this, goal_handle, feedback, result, velocity_publisher]() {
      // Check if goal is still active
      if (goal_handle->is_active()) {
        // Perform control loop operations
        if (goal_received_) {
          // Process the received goal
          // Implement the control algorithm to reach the desired point
          // Adjust the robot's velocity accordingly
          // Update the feedback with the progress

          // Publish the velocity command
          geometry_msgs::msg::Twist velocity_cmd;
          // Set the velocity values based on the control algorithm
          velocity_cmd.linear.x = 0.0;   // Adjust the linear velocity
          velocity_cmd.angular.z = 0.0;  // Adjust the angular velocity
          velocity_publisher->publish(velocity_cmd);

          // Update the feedback progress
          feedback->progress += 0.1;  // Update the progress value based on the control loop
          goal_handle->publish_feedback(feedback);  // Publish the feedback to the client
        }
      } else {
        // Goal is no longer active, perform any necessary cleanup or finalization
        if (result->success) {
          goal_handle->succeed(result);  // Indicate that the goal has succeeded
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
          goal_handle->abort(result);  // Indicate that the goal has failed or been aborted
          RCLCPP_INFO(this->get_logger(), "Goal aborted");
        }
      }
    });
  }

  rclcpp_action::Server<robot_interfaces::action::GoToPoint>::SharedPtr action_server_;
  bool goal_received_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPoint>());
  rclcpp::shutdown();
  return 0;
}
