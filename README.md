# manipulation

### simulation

```
source ~/ros2_ws/install/setup.bash
ros2 launch my_moveit_config move_group.launch.py
ros2 launch my_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place.launch.py
```

### Real robot

```
source ~/ros2_ws/install/setup.bash
ros2 launch my_moveit_config move_group.launch.py use_sim_time:=False trajectory_execution_yaml:=moveit_controllers_real.yaml
ros2 launch my_moveit_config moveit_rviz.launch.py
ros2 launch moveit2_scripts pick_and_place_real.launch.py
```