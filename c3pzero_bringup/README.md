# c3pzero_ignition

Ignition Gazebo simulation related launch, world and environment hooks for c3pzero robot. This package contains a single launch files:

- `c3pzero.launch.py`: Launches a given world file in Ignition Gazebo Simulator, defaults to empty_world.sdf. To launch the sim and not hardware:
```
ros2 launch c3pzero_ignition ignition.launch.py sim:=true
```
- The launch file launches the hardware if sim argument hasn't been set. To launch the hardware and drivers:
```
ros2 launch c3pzero_ignition spawn_robot.launch.py
```
