#!/usr/bin/env bash

## Determine the host directory to be mounted as a development volume
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"
DEV_VOLUME_HOST_DIR="${DEV_VOLUME_HOST_DIR:-"${REPOSITORY_DIR}"}"

## Determine the docker directory where the development volume will be mounted
DEV_VOLUME_DOCKER_DIR="${DEV_VOLUME_DOCKER_DIR:-"/root/ws_moveit/src/moveit_calibration"}"

## Run the docker container with the development volume mounted
echo -e "\033[2;37mDevelopment volume: ${DEV_VOLUME_HOST_DIR} -> ${DEV_VOLUME_DOCKER_DIR}\033[0m" | xargs
exec "${SCRIPT_DIR}/run.bash" -v "${DEV_VOLUME_HOST_DIR}:${DEV_VOLUME_DOCKER_DIR}" \
    -v "/home/andrej/ws/code/_robots/panda_ign_moveit2:/root/ws_moveit/src/panda_ign_moveit2" "${@}"
