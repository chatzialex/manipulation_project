import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    trajectory_execution_yaml_arg = LaunchConfiguration("trajectory_execution_yaml")
    
    trajectory_execution_yaml = os.path.join(get_package_share_directory('my_moveit_config'), 'config', trajectory_execution_yaml_arg.perform(context))

    # remap_args = []
    # if use_sim_time.perform(context).lower() == 'false':
    #    remap_args = [('/joint_states', '/merged_joint_states')]

    moveit_config = (
      MoveItConfigsBuilder("name", package_name="my_moveit_config")
          .trajectory_execution(file_path=trajectory_execution_yaml)
          .to_moveit_configs()
    )

    # print(f"moveit_config: {moveit_config.to_dict()}")

    return [
        generate_moveit_rviz_launch(moveit_config)
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
