#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "gtest/gtest.h"
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class ActionTestFixture : public ::testing::Test {
public:
  using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<WaypointAction>;
  ActionTestFixture() : done_(false) {
    action_Node = rclcpp::Node::make_shared("waypoint_action_client");
    client_ = rclcpp_action::create_client<WaypointAction>(action_Node,
                                                           "tortoisebot_as");

    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(action_Node->get_logger(),
                   "Action server not available after waiting");
      return;
    }
  }

  void send_goal(double x_, double y_, double yaw_) {
    auto goal_msg = WaypointAction::Goal();
    goal_msg.position.x = x_;
    goal_msg.position.y = y_;
    goal_msg.position.z = yaw_; // Desired yaw in radians
    goal_data_ = goal_msg.position;

    RCLCPP_INFO(action_Node->get_logger(),
                "Sending goal to move to (%f, %f) with yaw %f radians", x_, y_,
                yaw_);

    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionTestFixture::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&ActionTestFixture::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &ActionTestFixture::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleWaypoint::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(action_Node->get_logger(),
                   "Goal was rejected by the server");
    } else {
      RCLCPP_INFO(action_Node->get_logger(),
                  "Goal accepted by the server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleWaypoint::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {

    feedback_data_ = feedback->position;
  }

  void result_callback(const GoalHandleWaypoint::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(action_Node->get_logger(), "Goal succeeded!");
      done_ = true;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(action_Node->get_logger(), "Goal was canceled");
      done_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(action_Node->get_logger(), "Goal was aborted");
      done_ = true;
      break;
    default:
      RCLCPP_INFO(action_Node->get_logger(), "Unknown result code");
      break;
    }
  }

protected:
  geometry_msgs::msg::Point goal_data_, feedback_data_;
  bool done_;
  std::shared_ptr<rclcpp::Node> action_Node;
  rclcpp_action::Client<WaypointAction>::SharedPtr client_;
};

TEST_F(ActionTestFixture, TestPosition) {
  const double tolerance = 0.1;
  send_goal(2.0, 2.0, 1.57); // x, y, yaw_
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = 100s;
  while (!done_ && std::chrono::steady_clock::now() - start_time < timeout) {
    rclcpp::spin_some(action_Node);
  }
  EXPECT_TRUE(std::abs(feedback_data_.x - goal_data_.x) <= tolerance)
      << "X out of tolerance";
  EXPECT_TRUE(std::abs(feedback_data_.y - goal_data_.y) <= tolerance)
      << "y out of tolerance";
}

TEST_F(ActionTestFixture, TestOrientation) {
  const double tolerance = 0.1;
  send_goal(2.0, 2.0, 1.57); // x, y, yaw_
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = 30s;
  while (!done_ && std::chrono::steady_clock::now() - start_time < timeout) {
    rclcpp::spin_some(action_Node);
  }
  EXPECT_TRUE(std::abs(feedback_data_.z - goal_data_.z) <= tolerance)
      << "yaw out of tolerance";
}