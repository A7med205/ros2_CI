#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;

class WaypointActionClient : public rclcpp::Node {
public:
  using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<WaypointAction>;

  WaypointActionClient() : Node("waypoint_action_client") {
    client_ =
        rclcpp_action::create_client<WaypointAction>(this, "tortoisebot_as");

    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }

    send_goal();
  }

private:
  rclcpp_action::Client<WaypointAction>::SharedPtr client_;

  void send_goal() {
    auto goal_msg = WaypointAction::Goal();
    goal_msg.position.x = 1.0;
    goal_msg.position.y = 2.0;
    goal_msg.position.z = 1.57; // Desired yaw in radians

    RCLCPP_INFO(this->get_logger(),
                "Sending goal to move to (1.0, 2.0) with yaw 1.57 radians");

    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WaypointActionClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&WaypointActionClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &WaypointActionClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleWaypoint::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by the server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleWaypoint::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(),
                "Feedback received: Current position (%.2f, %.2f), Yaw: %.2f",
                feedback->position.x, feedback->position.y,
                feedback->position.z);
  }

  void result_callback(const GoalHandleWaypoint::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
