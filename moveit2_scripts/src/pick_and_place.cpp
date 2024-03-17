#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

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
    table_plane_pose.position.z = 0.0;

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
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

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
  target_pose1.position.x = 0.341;
  target_pose1.position.y = -0.020;
  target_pose1.position.z = 0.226;
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

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction{move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach)};

  if (fraction < 1.0) {
    RCLCPP_ERROR(
        LOGGER, "Failed to create approach trajectory (generated fraction=%f).",
        fraction);
    return 7;
  }

  if (move_group_arm.execute(trajectory_approach) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to follow approach trajectory.");
    return 8;
  }
  /*
  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("gripper_close");

  if (move_group_gripper.plan(my_plan_gripper) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to create plain to close gripper.");
    return 9;
  }

  if (move_group_gripper.execute(my_plan_gripper) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to close gripper.");
    return 10;
  }

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

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
                 "Failed to create retreat trajectory (generated fraction=%f).",
                 fraction);
    return 11;
  }

  if (move_group_arm.execute(trajectory_approach) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Failed to follow retreat trajectory.");
    return 12;
  }
  /*
  // Place

  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] = 1.57; // Shoulder Pan

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);
  */
  rclcpp::shutdown();
  return 0;
}