ARG ROS_DISTRO=humble
FROM moveit/moveit2:${ROS_DISTRO}-source

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Install Gazebo
ARG GZ_VERSION=fortress
ENV GZ_VERSION=${GZ_VERSION}
ARG IGNITION_VERSION=${GZ_VERSION}
ENV IGNITION_VERSION=${GZ_VERSION}
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    gz-${GZ_VERSION} && \
    rm -rf /var/lib/apt/lists/*

### Import and install dependencies, then build these dependencies (not moveit_calibration yet)
COPY ./moveit_calibration_demos/moveit_calibration_demos.repos /root/ws_moveit/src/moveit_calibration/moveit_calibration_demos/moveit_calibration_demos.repos
RUN vcs import --shallow /root/ws_moveit/src < /root/ws_moveit/src/moveit_calibration/moveit_calibration_demos/moveit_calibration_demos.repos && \
    rosdep update && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths /root/ws_moveit/src && \
    rm -rf /var/lib/apt/lists/* && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "/root/ws_moveit/install/local_setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --ament-cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /root/ws_moveit/log

### Copy over the rest of moveit_calibration, then install dependencies and build
COPY ./ /root/ws_moveit/src/moveit_calibration/
RUN rosdep update && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths /root/ws_moveit/src && \
    rm -rf /var/lib/apt/lists/* && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "/root/ws_moveit/install/local_setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --ament-cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /root/ws_moveit/log

### Install panda packages again with --symlink-install (workaround, needed for SDF exporter)
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "/root/ws_moveit/install/local_setup.bash" && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --ament-cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select panda_description panda_moveit_config panda --symlink-install && \
    rm -rf /root/ws_moveit/log

### Add workspace to the ROS entrypoint
### Source ROS workspace inside `~/.bashrc` to enable autocompletion
RUN sed -i '$i source "/root/ws_moveit/install/local_setup.bash" --' /ros_entrypoint.sh && \
    sed -i '$a source "/opt/ros/${ROS_DISTRO}/setup.bash"' ~/.bashrc
