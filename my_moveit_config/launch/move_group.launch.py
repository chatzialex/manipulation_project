import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    trajectory_execution_yaml_arg = LaunchConfiguration("trajectory_execution_yaml")
    
    trajectory_execution_yaml = os.path.join(get_package_share_directory('my_moveit_config'), 'config', trajectory_execution_yaml_arg.perform(context))

    moveit_config = (
      MoveItConfigsBuilder("name", package_name="my_moveit_config")
          .trajectory_execution(file_path=trajectory_execution_yaml)
          .to_moveit_configs()
    )

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
                {"publish_robot_description_semantic": True},
                {"use_sim_time": use_sim_time},
            ],
        )
    ]

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    trajectory_execution_yaml_arg = DeclareLaunchArgument(
        "trajectory_execution_yaml",
        default_value="moveit_controllers_sim.yaml"
    )

    return LaunchDescription([
        use_sim_time_arg,
        trajectory_execution_yaml_arg,
        OpaqueFunction(function=launch_setup)
    ])