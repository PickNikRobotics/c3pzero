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

Linting
```shell
username@c3pzero_siteconfig-dev:~/ws$ pre-commit run -a
```

# Requirements
- docker compose 2.6.0
  - https://docs.docker.com/compose/install/compose-plugin/#install-the-plugin-manually

# Packages

We currently have the following packages, please refer to their individual README files for launch files and more information:

 - [c3pzero_navigation](c3pzero_navigation/README.md)

## References
- [Development container documentation](docs/development-container.md)
