# 3D party pkgs - Docker

[![Build docker images](https://github.com/TW-Robotics/taurob_tracker/actions/workflows/build_docker.yml/badge.svg?branch=master&event=push)](https://github.com/TW-Robotics/taurob_tracker/actions/workflows/build_docker.yml)

Contains Dockerfiles, build & run scripts as well as docker-compose configurations of 3D party ROS pkgs

## General Usage
* __Automatic__:[👑] The containers are started automatically using [systemd](./systemd/Readme.md) on boot [_recommended for autostart_]. 
* __Manual__:[🚫] The docker container can either be build/started either individually (using bash scripts) or simultaneously (using docker compose) [_only use for testing_].

### Docker-Compose
Is the recommended way of starting multiple Docker Container at once.


__Build:__
Builds all containers as defined in the compose yaml files into one application.
Usage: `docker-compose -f docker-compose.yaml build`


__RUN:__   
Usage: `docker-compose -f docker-compose.yaml up`


### Bash Scripts
The bash scripts can be used to test docker container individually, but docker-compose is the recommended way of starting.

__Build:__   
All `build_docker_<name>.sh` files build an image of the corresponding `Dockerfile.<name>`.   
Usage: `bash build_docker_<name>.sh`


__RUN:__   
Run scripts can be used to start one image. The logs will outomatically be stored in `<path-to-run-script>/logs/<container-name>.log`
Usage: `bash run_docker_<name>.sh`
