from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  use_sim_time = LaunchConfiguration("use_sim_time")
  
  use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time",
     default_value="True"
  )

  basic_grasping_perception_node = Node(
    package="simple_grasping",
    executable="basic_grasping_perception_node",
    output="screen",
    parameters = [{"use_sim_time" : use_sim_time}],
    emulate_tty=True
  )

  get_cube_pose_node = Node(
    package="get_cube_pose",
    executable="get_cube_pose",
    output="screen",
    parameters = [{"use_sim_time" : use_sim_time}],
    emulate_tty=True
  )

  return LaunchDescription([
    use_sim_time_arg,
    basic_grasping_perception_node,
    get_cube_pose_node
  ])  