#!/usr/bin/env bash

## Configuration
# Default Docker Hub user and repository name (used if inferred image is not available)
DEFAULT_DOCKERHUB_USER="andrejorsula"
DEFAULT_REPOSITORY_NAME="moveit2_calibration"
# Flags for running the container
DOCKER_RUN_OPTS="${DOCKER_RUN_OPTS:-
    --name "${DEFAULT_REPOSITORY_NAME}"
    --interactive
    --tty
    --rm
    --network host
    --ipc host
}"
# Flags for enabling GPU (NVIDIA) and GUI (X11) inside the container
ENABLE_GPU="${ENABLE_GPU:-true}"
ENABLE_GUI="${ENABLE_GUI:-true}"
# List of volumes to mount (can be updated by passing -v HOST_DIR:DOCKER_DIR:OPTIONS)
CUSTOM_VOLUMES=(
    "/etc/localtime:/etc/localtime:ro"
)
# List of environment variables to set (can be updated by passing -e ENV=VALUE)
CUSTOM_ENVS=(
)

## Determine the name of the image to run (automatically inferred from the current user and repository, or using the default if not available)
# Get the current Docker Hub user or use the default
DOCKERHUB_USER="$(docker info | sed '/Username:/!d;s/.* //')"
DOCKERHUB_USER="${DOCKERHUB_USER:-${DEFAULT_DOCKERHUB_USER}}"
# Get the name of the repository (directory) or use the default
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"
if [[ -f "${REPOSITORY_DIR}/Dockerfile" ]]; then
    REPOSITORY_NAME="$(basename "${REPOSITORY_DIR}")"
else
    REPOSITORY_NAME="${DEFAULT_REPOSITORY_NAME}"
fi
# Combine the user and repository name to form the image name
IMAGE_NAME="${DOCKERHUB_USER}/${REPOSITORY_NAME}"
# Determine if such image exists (either locally or on Docker Hub), otherwise use the default image name
if [[ -z "$(docker images -q "${IMAGE_NAME}")" ]] || [[ -z "$(wget -q "https://registry.hub.docker.com/v2/repositories/${IMAGE_NAME}" -O -)" ]]; then
    IMAGE_NAME="${DEFAULT_DOCKERHUB_USER}/${DEFAULT_REPOSITORY_NAME}"
fi

## Parse volumes and environment variables
while getopts ":v:e:" opt; do
    case "${opt}" in
        v) CUSTOM_VOLUMES+=("${OPTARG}") ;;
        e) CUSTOM_ENVS+=("${OPTARG}") ;;
        *)
            echo >&2 "Usage: ${0} [-v HOST_DIR:DOCKER_DIR:OPTIONS] [-e ENV=VALUE] [TAG] [CMD]"
            exit 2
            ;;
    esac
done
shift "$((OPTIND - 1))"

## Parse TAG and CMD positional arguments
if [ "${#}" -gt "0" ]; then
    if [[ $(docker images --format "{{.Tag}}" "${IMAGE_NAME}") =~ (^|[[:space:]])${1}($|[[:space:]]) || $(wget -q "https://registry.hub.docker.com/v2/repositories/${IMAGE_NAME}/tags" -O - | grep -Poe '(?<=(\"name\":\")).*?(?=\")') =~ (^|[[:space:]])${1}($|[[:space:]]) ]]; then
        # Use the first argument as a tag is such tag exists either locally or on the remote registry
        IMAGE_NAME="${IMAGE_NAME}:${1}"
        CMD=${*:2}
    else
        CMD=${*:1}
    fi
fi

## GPU
if [[ "${ENABLE_GPU}" = true ]]; then
    LS_HW_DISPLAY=$(lshw -C display 2> /dev/null | grep vendor)
    if [[ ${LS_HW_DISPLAY^^} =~ NVIDIA ]]; then
        # Enable GPU either via NVIDIA Container Toolkit or NVIDIA Docker (depending on Docker version)
        if dpkg --compare-versions "$(docker version --format '{{.Server.Version}}')" gt "19.3"; then
            GPU_OPT="--gpus all"
        else
            GPU_OPT="--runtime nvidia"
        fi
        GPU_ENVS=(
            NVIDIA_VISIBLE_DEVICES="all"
            NVIDIA_DRIVER_CAPABILITIES="all"
        )
    fi
fi

## GUI
if [[ "${ENABLE_GUI}" = true ]]; then
    # To enable GUI, make sure processes in the container can connect to the x server
    XAUTH=/tmp/.docker.xauth
    if [ ! -f ${XAUTH} ]; then
        touch ${XAUTH}
        chmod a+r ${XAUTH}

        XAUTH_LIST=$(xauth nlist "${DISPLAY}")
        if [ -n "${XAUTH_LIST}" ]; then
            # shellcheck disable=SC2001
            XAUTH_LIST=$(sed -e 's/^..../ffff/' <<<"${XAUTH_LIST}")
            echo "${XAUTH_LIST}" | xauth -f ${XAUTH} nmerge -
        fi
    fi
    # GUI-enabling volumes
    GUI_VOLUMES=(
        "${XAUTH}:${XAUTH}"
        "/tmp/.X11-unix:/tmp/.X11-unix"
        "/dev/input:/dev/input"
    )
    # GUI-enabling environment variables
    GUI_ENVS=(
        DISPLAY="${DISPLAY}"
        XAUTHORITY="${XAUTH}"
    )
fi

## Run the container
DOCKER_RUN_CMD=(
    docker run
    "${DOCKER_RUN_OPTS}"
    "${GPU_OPT}"
    "${GPU_ENVS[@]/#/"--env "}"
    "${GUI_VOLUMES[@]/#/"--volume "}"
    "${GUI_ENVS[@]/#/"--env "}"
    "${CUSTOM_VOLUMES[@]/#/"--volume "}"
    "${CUSTOM_ENVS[@]/#/"--env "}"
    "${IMAGE_NAME}"
    "${CMD}"
)
echo -e "\033[1;30m${DOCKER_RUN_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
exec ${DOCKER_RUN_CMD[*]}
