#pragma once

#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <optional>

class FindObjectsClient : public rclcpp::Node {
public:
  explicit FindObjectsClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

  const std::optional<geometry_msgs::msg::Pose> &getResult() const {
    return result_;
  }

  void sendGoal();

private:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  rclcpp_action::Client<Find>::SharedPtr client_ptr_{};

  bool in_progress_{false};
  std::optional<geometry_msgs::msg::Pose> result_{};

  void goalResponseCallback(const GoalHandleFind::SharedPtr &goal_handle);

  void
  feedbackCallback(GoalHandleFind::SharedPtr,
                   const std::shared_ptr<const Find::Feedback> /*feedback*/);

  void resultCallback(const GoalHandleFind::WrappedResult &result);
}; // class FindGoalsClient