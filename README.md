# Manipulation

### Simulation

```
source ~/ros2_ws/install/setup.bash
ros2 launch my_moveit_config move_group.launch.py
ros2 launch my_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place.launch.py
```

### Real Robot

```
source ~/ros2_ws/install/setup.bash
ros2 launch my_moveit_config move_group.launch.py use_sim_time:=False trajectory_execution_yaml:=moveit_controllers_real.yaml
ros2 launch my_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place_real.launch.py
```

# Manipulation with perception

### Simulation

```
ros2 launch get_cube_pose get_pose_client.launch.py
```

### Real Robot

```
ros2 launch get_cube_pose get_pose_client.launch.py use_sim_time:=False
```

# Known issues
- The IK solver exhibits strange behavior in simulation. It was observed to sometimes try to rapidly switch between +=360 degree joint configurations during the approach/retract phases. Currently this is disallowed by the jump_threshold parameter, which means sometimes the solver fails during these phases. With the default KDL solver this behavior is much more exagerated and it fails completely in simulation.
- Cube gets knocked when grasping in simulation. Currently a multi-stage grasping approach is implemented to mitigate this issue, but even after a lot of tuning, it fails to work reliably.
- MoveIt2 doesn't receive the gripper joint states on the real robot. It is able to perform open/close gripper movements, but it thinks it fais nevertheless. The gripper state is also not visualized correctly in Rviz as a consequence. The correct gripper state is available on the /gripper/joint_states topic. It would, it principle, be possible to aggregate the joint states from the arm and gripper in a single topic using a joint_state_publisher node. This is, however, prevented by the fact that the wrong gripper state is published on the /joint_states topic. Thus, the only viable solution to aggregate all joint states in a single topic is by writing a custom "republisher" node, which is rather not clean and thus not implemented. 
- The robot name "ur3e" used on the real robot setup is incompatibly with the name "name" used in Moveit2 (coming from the URDF?). This results in warnings during experiments on the real robot.