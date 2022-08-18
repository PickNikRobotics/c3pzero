# c3pzero_ignition

Ignition Gazebo simulation related launch, world and environment hooks for c3pzero robot. This package contains two launch files:

- ignition.launch.py: Launches a given world file in Ignition Gazebo Simulator, defaults to empty_world.sdf. To launch:
```
ros2 launch c3pzero_ignition ignition.launch.py world:=empty
```
- spawn_robot.launch.py: Launches the c3pzero robot with all the sensors and controllers configured. Users need to launch a Ignition Gazebo (as in the previous command) instance to spawn the robot. To launch:
```
ros2 launch c3pzero_ignition ignition.launch.py rviz:=true
```
