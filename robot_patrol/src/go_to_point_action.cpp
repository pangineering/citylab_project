#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

#include "rclcpp_action/server_goal_handle.hpp"
#include "robot_interfaces/action/go_to_point.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoToPointServer : public rclcpp::Node {
public:
  using GoToPoint = robot_interfaces::action::GoToPoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPoint>;

  explicit GoToPointServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_point_server", options) {
    using namespace std::placeholders;

    // Create a new action server for the GoToPoint action
    action_server_ = rclcpp_action::create_server<GoToPoint>(
        this, "go_to_point",
        // Bind the handle_goal function to the server
        std::bind(&GoToPointServer::handle_goal, this, _1, _2),
        // Bind the handle_cancel function to the server
        std::bind(&GoToPointServer::handle_cancel, this, _1),
        // Bind the handle_accepted function to the server
        std::bind(&GoToPointServer::handle_accepted, this, _1));

    // Subscribe to the odometry topic
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPointServer::odometry_callback, this, _1));

    // Create a publisher for velocity commands
    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create a timer for the control loop
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GoToPointServer::control_loop, this));
  }

private:
  double robot_position_x_;
  double robot_position_y_;
  double robot_heading_;
  double distance_threshold_;

  rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position [%.2f, %.2f, %.2f]",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    using namespace std::placeholders;

    // Start a new thread to execute the goal
    std::thread{std::bind(&GoToPointServer::execute_goal, this, _1),
                goal_handle}
        .detach();
  }

  void execute_goal(const std::shared_ptr<GoalHandle> goal_handle) {
    // Get the goal position from the goal handle
    const auto goal_pos = goal_handle->get_goal()->goal_pos;

    // Start publishing feedback at a fixed rate
    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
      // Check if the goal has been canceled
      if (goal_handle->is_canceling()) {
        // Mark the goal as canceled and return
        /// goal_handleabort();
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Publish feedback with the current position
      auto feedback_msg = std::make_shared<GoToPoint::Feedback>();
      feedback_msg->current_pos = getCurrentPosition();
      goal_handle->publish_feedback(feedback_msg);

      // Check if the goal has been achieved
      if (isGoalAchieved(goal_pos)) {
        // Mark the goal as succeeded and return
        goal_handle->succeed(std::make_shared<GoToPoint::Result>());
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }

      // Sleep for the feedback publishing rate
      rate.sleep();
    }
  }

  geometry_msgs::msg::Point32 getCurrentPosition() {
    geometry_msgs::msg::Point32 current_pos;
    current_pos.x = robot_position_x_;
    current_pos.y = robot_position_y_;
    current_pos.z = robot_heading_;
    return current_pos;
  }

  bool isGoalAchieved(const geometry_msgs::msg::Point32 &goal_pos) {
    // Implement your logic to determine if the goal has been achieved
    // Compare the current position with the goal position and check the
    // distance threshold Return true if the goal is achieved, false otherwise
    return false; // Placeholder value, replace with your implementation
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Process odometry data received from the subscription
    // Update the internal state or variables used in the control loop

    // Access relevant odometry information from the msg object
    double robot_x = msg->pose.pose.position.x;
    double robot_y = msg->pose.pose.position.y;
    double robot_yaw = 0.0; // Calculate robot's yaw or heading angle

    // Update the internal state or variables used in the control loop
    robot_position_x_ = robot_x;
    robot_position_y_ = robot_y;
    robot_heading_ = robot_yaw;
  }

  void control_loop() {
    // Perform control actions based on the current state and goal

    // Calculate the control actions based on the current state and goal
    // For example, you can use a PID controller or any other control algorithm

    // Example control actions:
    double linear_velocity = 0.5;  // Set the desired linear velocity
    double angular_velocity = 0.2; // Set the desired angular velocity

    // Publish velocity commands to the robot
    geometry_msgs::msg::Twist velocity_cmd;
    velocity_cmd.linear.x = linear_velocity;
    velocity_cmd.angular.z = angular_velocity;
    velocity_publisher_->publish(velocity_cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPointServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
