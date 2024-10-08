cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pick_and_place
    src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_executable(pick_and_place_real
    src/pick_and_place_real.cpp)
ament_target_dependencies(pick_and_place_real
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs)

add_library(find_objects_client
	src/find_objects_client.cpp)
target_include_directories(find_objects_client
	PUBLIC
	include
)
ament_target_dependencies(find_objects_client
	rclcpp
	rclcpp_action
	grasping_msgs
	geometry_msgs
)

add_executable(pick_and_place_perception
    src/pick_and_place_perception.cpp)
target_link_libraries(pick_and_place_perception
	find_objects_client
)
ament_target_dependencies(pick_and_place_perception
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
)

add_executable(pick_and_place_perception_real
    src/pick_and_place_perception_real.cpp)
target_link_libraries(pick_and_place_perception_real
	find_objects_client
)
ament_target_dependencies(pick_and_place_perception_real
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	grasping_msgs
)

# Install the executable
install(TARGETS 
	pick_and_place
	pick_and_place_real
	pick_and_place_perception
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
