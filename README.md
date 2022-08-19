# c3pzero_siteconfig
MoveIt Studio Site Configuration for navigation with c3pzero mobile base

# Build
Build a new development image
```shell
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

The following ports are used to communicate with hardware
- `/dev/ttyACM0` for motor driver
- `/dev/ttyUSB0` for lidar

Run these commands to allow the drivers to talk to the hardware
```
sudo chmod 666 /dev/ttyACM0
sudo chown $USER /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0
sudo chown $USER /dev/ttyUSB0
sudo adduser $USER dialout
```


Linting
```shell
username@c3pzero_siteconfig-dev:~/ws$ pre-commit run -a
```

# Requirements
- docker compose 2.6.0
  - https://docs.docker.com/compose/install/compose-plugin/#install-the-plugin-manually

# Packages

We currently have the following packages, please refer to their individual README files for launch files and more information:

- [c3pzero_description](c3pzero_description/README.md)
- [c3pzero_ignition](c3pzero_ignition/README.md)
- [c3pzero_navigation](c3pzero_navigation/README.md)
- [c3pzero_bringup](c3pzero_bringup/README.md)

## References
- [Development container documentation](docs/development-container.md)
