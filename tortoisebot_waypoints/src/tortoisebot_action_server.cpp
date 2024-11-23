#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tortoisebot_waypoints/action/waypoint_action.hpp>

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;

class WaypointActionServer : public rclcpp::Node {
public:
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<WaypointAction>;

  WaypointActionServer() : Node("tortoisebot_action_server") {
    action_server_ = rclcpp_action::create_server<WaypointAction>(
        this, "tortoisebot_as",
        std::bind(&WaypointActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&WaypointActionServer::handle_accepted, this,
                  std::placeholders::_1));

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&WaypointActionServer::odom_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waypoint Action Server started");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received goal cancellation request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::execute, this, goal_handle)}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();

    geometry_msgs::msg::Point target_position = goal->position;
    double err_pos =
        std::sqrt(std::pow(target_position.y - current_position_.y, 2) +
                  std::pow(target_position.x - current_position_.x, 2));
    double err_yaw = std::atan2(target_position.y - current_position_.y,
                                target_position.x - current_position_.x) -
                     current_yaw_;
    bool orientation_ = false;

    while (rclcpp::ok() && (err_pos > dist_precision_ || !orientation_)) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        goal_handle->canceled(result);
        return;
      }

      geometry_msgs::msg::Twist twist;

      if (err_pos > dist_precision_) {
        // If position error is significant, move towards the target
        err_yaw = std::atan2(target_position.y - current_position_.y,
                             target_position.x - current_position_.x) -
                  current_yaw_;
        if (std::abs(err_yaw) > yaw_precision_) {
          twist.angular.z = (err_yaw > 0 ? 0.5 : -0.5);
          RCLCPP_INFO(this->get_logger(), "Fixing yaw to face the target");
        } else {
          twist.linear.x = 0.65;
          twist.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "Moving towards the target");
        }
      } else {
        // When tolerance reached, adjust yaw to the desired final orientation
        err_yaw = target_position.z - current_yaw_;
        if (std::abs(err_yaw) > yaw_precision_) {
          twist.angular.z = (err_yaw > 0 ? 0.5 : -0.5);
          twist.linear.x = 0.0;
          RCLCPP_INFO(this->get_logger(),
                      "Adjusting final yaw to desired orientation");
        } else {
          twist.angular.z = 0.0;
          twist.linear.x = 0.0;
          orientation_ = true;
          RCLCPP_INFO(this->get_logger(),
                      "Target reached with final orientation");
        }
      }

      cmd_vel_publisher_->publish(twist);

      // Publish feedback
      feedback->position = current_position_;
      feedback->position.z = current_yaw_;
      goal_handle->publish_feedback(feedback);

      rclcpp::Rate(25).sleep();

      // Update errors
      err_pos = std::sqrt(std::pow(target_position.y - current_position_.y, 2) +
                          std::pow(target_position.x - current_position_.x, 2));
    }

    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_publisher_->publish(stop_twist);

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  geometry_msgs::msg::Point current_position_;
  double current_yaw_ = 0.0;
  double yaw_precision_ = M_PI / 90; // +/- 2 degrees
  double dist_precision_ = 0.05;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
