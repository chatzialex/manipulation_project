from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    basic_grasping_perception_node = Node(
        package="simple_grasping",
        executable="basic_grasping_perception_node",
        output="screen",
        parameters = [{"use_sim_time" : True}],
        emulate_tty=True
    )

    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        basic_grasping_perception_node,
        moveit_cpp_node
    ])