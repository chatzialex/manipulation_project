#include "moveit2_scripts/find_objects_client.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <inttypes.h>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

FindObjectsClient::FindObjectsClient(const rclcpp::NodeOptions &node_options)
    : Node{"find_objects_client", node_options},
      client_ptr_{rclcpp_action::create_client<Find>(
          this->get_node_base_interface(), this->get_node_graph_interface(),
          this->get_node_logging_interface(),
          this->get_node_waitables_interface(), "find_objects")} {}

void FindObjectsClient::sendGoal() {
  using namespace std::placeholders;

  if (in_progress_) {
    RCLCPP_ERROR(this->get_logger(), "Another goal is in progress, returning.");
    return;
  }
  in_progress_ = true;
  result_ = std::nullopt;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    in_progress_ = false;
    return;
  }
  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    in_progress_ = false;
    return;
  }

  auto goal_msg = Find::Goal();
  goal_msg.plan_grasps = false;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&FindObjectsClient::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&FindObjectsClient::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&FindObjectsClient::resultCallback, this, _1);
  auto goal_handle_future =
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void FindObjectsClient::goalResponseCallback(
    const GoalHandleFind::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    in_progress_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void FindObjectsClient::feedbackCallback(
    GoalHandleFind::SharedPtr,
    const std::shared_ptr<const Find::Feedback> /*feedback*/) {
  RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
}

void FindObjectsClient::resultCallback(
    const GoalHandleFind::WrappedResult &result) {
  in_progress_ = false;
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  result_ = result.result->objects[0].object.primitive_poses[0];
  /*for (auto object : result.result->objects) {
    RCLCPP_INFO(this->get_logger(), "Type: %d",
                object.object.primitives[0].type);
    RCLCPP_INFO(this->get_logger(), "x: %f",
                object.object.primitives[0].dimensions[0]);
    RCLCPP_INFO(this->get_logger(), "Y: %f",
                object.object.primitives[0].dimensions[1]);
    RCLCPP_INFO(this->get_logger(), "Z: %f",
                object.object.primitives[0].dimensions[2]);
  }*/
}