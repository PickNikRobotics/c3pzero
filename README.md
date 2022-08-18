# c3pzero_siteconfig
MoveIt Studio Site Configuration for navigation with c3pzero mobile base

# Build
Build a new development image
```shell
mkdir -p ~/.c3pzero_siteconfig/ccache
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml build --ssh default
```
Start an interactive development container
```shell
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
username@c3pzero_siteconfig-dev:~/ws$ colcon build
```

# Requirements
- docker compose 2.6.0
  - https://docs.docker.com/compose/install/compose-plugin/#install-the-plugin-manually

# Capabilities and Packages

We currently have the following launch files:

view_robot_base.launch.py: Launches the robot urdf for viewing in RViz. To launch:

  ros2 launch c3pzero_description view_robot_base.launch.py

ignition.launch.py: Launches a given world file in Ignition Gazebo Simulator, defaults to empty_world.sdf. To launch:

  ros2 launch c3pzero_ignition ignition.launch.py world:=empty

spawn_robot.launch.py: Launches the c3pzero robot with all the sensors and controllers configured. Users need to launch a Ignition Gazebo (as in the previous command) instance to spawn the robot. To launch:

  ros2 launch c3pzero_ignition ignition.launch.py rviz:=true

## References
- [Development container documentation](docs/development-container.md)
