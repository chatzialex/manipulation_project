#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// Gripper values.
constexpr double kGripperGraspingValue{0.656};
/*
constexpr double kObjectWidth{0.40};
static const double kGripperGraspingValue{
    (-140840.9278) * std::pow(kObjectWidth, 5.0) +
    (+22667.9431) * std::pow(kObjectWidth, 4.0) +
    (-1551.6277) * std::pow(kObjectWidth, 3.0) +
    (+38.2691) * std::pow(kObjectWidth, 2.0) +
    (-8.0630) * std::pow(kObjectWidth, 1.0) +
    (+0.8047) * std::pow(kObjectWidth, 0.0)};
*/
constexpr double kGripperMinValue{0.000}; // rad
constexpr double kGripperMaxValue{0.800}; // rad
constexpr double kGripperTolerance{0.01}; // rad
constexpr int kGripperGraspStages{3};

int main(int argc, char **argv) {
  // Initialize.
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  RCLCPP_INFO(LOGGER, "Arm planning frame: %s",
              move_group_arm.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "Arm end effector link: %s",
              move_group_arm.getEndEffectorLink().c_str());

  // Setup collision objects.

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive wall_plane;

    // Define the wall plane
    wall_plane.type = wall_plane.BOX;
    wall_plane.dimensions.resize(3);
    wall_plane.dimensions[wall_plane.BOX_X] = 2.0;
    wall_plane.dimensions[wall_plane.BOX_Y] = 0.01;
    wall_plane.dimensions[wall_plane.BOX_Z] = 2.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose wall_plane_pose;
    wall_plane_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    wall_plane_pose.position.x = 0.0;
    wall_plane_pose.position.y = 0.5;
    wall_plane_pose.position.z = 0.0;

    collision_object.primitives.push_back(wall_plane);
    collision_object.primitive_poses.push_back(wall_plane_pose);
    collision_object.operation = collision_object.ADD;

    // Define the table plane
    shape_msgs::msg::SolidPrimitive table_plane;
    table_plane.type = table_plane.BOX;
    table_plane.dimensions.resize(3);
    table_plane.dimensions[table_plane.BOX_X] = 2.0;
    table_plane.dimensions[table_plane.BOX_Y] = 2.0;
    table_plane.dimensions[table_plane.BOX_Z] = 0.01;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose table_plane_pose;
    table_plane_pose.orientation.w =
        1.0; // We can leave out the x, y, and z components of the quaternion
             // since they are initialized to 0
    table_plane_pose.position.x = 0.5;
    table_plane_pose.position.y = 0.25;
    table_plane_pose.position.z = -0.011;

    collision_object.primitives.push_back(table_plane);
    collision_object.primitive_poses.push_back(table_plane_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;

  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();
  move_group_arm.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  if (move_group_arm.plan(my_plan_arm) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to home position.");
    return 1;
  }

  if (move_group_arm.execute(my_plan_arm) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to move to home position.");
    return 2;
  }

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  // move_group_arm.setStartStateToCurrentState();
  // move_group_gripper.setStartStateToCurrentState();

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -0.707;
  target_pose1.orientation.y = 0.707;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.34;
  target_pose1.position.y = -0.020;
  target_pose1.position.z = 0.24;
  move_group_arm.setPoseTarget(target_pose1);

  if (move_group_arm.plan(my_plan_arm) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to pregrasp position.");
    return 3;
  }

  if (move_group_arm.execute(my_plan_arm) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to move to pregrasp position.");
    return 4;
  }

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  if (move_group_gripper.plan(my_plan_gripper) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to create plain to open gripper.");
    return 5;
  }

  if (move_group_gripper.execute(my_plan_gripper) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper.");
    return 6;
  }

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  move_group_arm.setStartStateToCurrentState();
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 10.0;
  const double eef_step = 0.01;

  double fraction{move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach)};

  if (fraction < 1.0) {
    RCLCPP_ERROR(LOGGER,
                 "Failed to create approach trajectory (only able to generate "
                 "%f %% of it).",
                 fraction * 100);
    return 7;
  }

  RCLCPP_INFO(LOGGER, "Generated %f%% of the desired path).", fraction * 100);

  if (move_group_arm.execute(trajectory_approach) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to follow approach trajectory.");
    return 8;
  }

  // Close Gripper
  RCLCPP_INFO(LOGGER, "Close Gripper!");

  std::vector<double> joint_group_positions_gripper;
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  // multi stage gripper close action for safe gripping of object
  auto gripper_value{kGripperGraspingValue -
                     kGripperGraspStages * kGripperTolerance};
  bool successful_grasp{true};
  for (int i{0}; i < kGripperGraspStages; ++i) {
    gripper_value += kGripperTolerance;
    current_state_gripper->copyJointGroupPositions(
        joint_model_group_gripper, joint_group_positions_gripper);
    joint_group_positions_gripper[2] = gripper_value;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    RCLCPP_INFO(LOGGER, "Grasping width: %f", joint_group_positions_gripper[2]);
    successful_grasp &= move_group_gripper.plan(my_plan_gripper) ==
                            moveit::core::MoveItErrorCode::SUCCESS &&
                        move_group_gripper.execute(my_plan_gripper) ==
                            moveit::core::MoveItErrorCode::SUCCESS;
    if (!successful_grasp) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }

  if (!successful_grasp) {
    RCLCPP_ERROR(LOGGER, "Failed to grasp the object.");
    return 9;
  }

  std::this_thread::sleep_for(500ms);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");
  move_group_arm.setStartStateToCurrentState();
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.03;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  if (fraction < 1.0) {
    RCLCPP_ERROR(LOGGER,
                 "Failed to create approach trajectory (only able to generate "
                 "%f %% of it).",
                 fraction * 100);
    return 11;
  }

  if (move_group_arm.execute(trajectory_retreat) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to follow retreat trajectory.");
    return 12;
  }

  // Place

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  const auto shoulder_lower_limit{
      joint_model_group_arm->getActiveJointModelsBounds()[0][0][0]
          .min_position_};

  if (joint_group_positions_arm[0] - 3.14 >= shoulder_lower_limit) {
    joint_group_positions_arm[0] -= 3.14;
  } else {
    joint_group_positions_arm[0] += 3.14;
  }

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  if (move_group_arm.plan(my_plan_arm) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to release position.");
    return 13;
  }

  if (move_group_arm.execute(my_plan_arm) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to move to release position.");
    return 14;
  }

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("gripper_open");

  if (move_group_gripper.plan(my_plan_gripper) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to create plain to open gripper.");
    return 15;
  }

  if (move_group_gripper.execute(my_plan_gripper) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper.");
    return 16;
  }

  rclcpp::shutdown();
  return 0;
}