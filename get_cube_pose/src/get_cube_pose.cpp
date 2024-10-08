#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

class GetCubePose : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit GetCubePose(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node{"get_cube_pose", node_options},
        client_ptr_{rclcpp_action::create_client<Find>(
            this->get_node_base_interface(), this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), "find_objects")},
        timer_{
            this->create_wall_timer(std::chrono::milliseconds(500),
                                    std::bind(&GetCubePose::sendGoal, this))},
        goal_done_{false} {}

  bool isGoalDone() const { return this->goal_done_; }

  void sendGoal() {
    using namespace std::placeholders;

    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GetCubePose::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&GetCubePose::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GetCubePose::resultCallback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_{};
  rclcpp::TimerBase::SharedPtr timer_{};
  bool goal_done_{};

  void goalResponseCallback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void
  feedbackCallback(GoalHandleFind::SharedPtr,
                   const std::shared_ptr<const Find::Feedback> /*feedback*/) {
    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  }

  void resultCallback(const GoalHandleFind::WrappedResult &result) {
    this->goal_done_ = true;
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
    for (auto object : result.result->objects) {
      RCLCPP_INFO(this->get_logger(), "Type: %d",
                  object.object.primitives[0].type);
      RCLCPP_INFO(this->get_logger(), "x: %f",
                  object.object.primitive_poses[0].position.x);
      RCLCPP_INFO(this->get_logger(), "y: %f",
                  object.object.primitive_poses[0].position.y);
      RCLCPP_INFO(this->get_logger(), "z: %f",
                  object.object.primitive_poses[0].position.z);
    }
    //}
  }
}; // class GetCubePose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetCubePose>();

  while (!action_client->isGoalDone()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}