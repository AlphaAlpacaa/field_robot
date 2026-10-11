# Field robot simulation - ROS 2 Jazzy + Gazebo Harmonic
#
# Built on the official ros-base image and filled out with the exact packages
# this project uses, so a clone plus `docker compose up` reproduces the
# environment with nothing installed on the host.
#
# One image, one layer of apt: splitting the installs into several RUN lines
# looks tidier but costs a full re-download whenever an early one changes.

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# The user inside the container must share the host user's numeric id, or
# every file written into the mounted workspace comes back owned by root and
# cannot be edited outside the container. The defaults suit the first user
# account on a desktop Ubuntu; docker/run.sh passes the real ones.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y \
    \
    # desktop tooling: rviz2, rqt, tf2 tools, the usual message packages
    ros-jazzy-desktop \
    \
    # Gazebo Harmonic and the ROS bridge. The vendor package pulls in the
    # simulator itself, so Gazebo is never installed separately.
    ros-jazzy-ros-gz \
    \
    # ros2_control: the arm and gripper controllers, and the plugin that
    # hosts controller_manager inside the Gazebo process
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-controller-manager \
    \
    # motion planning
    ros-jazzy-moveit \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-visual-tools \
    \
    # the UR5e model, its kinematics and its MoveIt defaults
    ros-jazzy-ur-description \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-ur-controllers \
    \
    # robot description and driving
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-twist-mux \
    \
    # perception
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-vision-opencv \
    ros-jazzy-rqt-image-view \
    python3-opencv \
    \
    # navigation - not used yet, but the next step in this project, and
    # adding it now spares a rebuild later
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    \
    # build and inspection tools
    python3-colcon-common-extensions \
    liburdfdom-tools \
    mesa-utils \
    x11-apps \
    git nano vim less \
    && rm -rf /var/lib/apt/lists/*

# Create the user, unless an account already holds that id (some base images
# ship one). Give it passwordless sudo: apt-installing a package while
# debugging should not mean rebuilding the image.
RUN if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi \
 && if ! getent passwd ${USER_UID} >/dev/null; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    else \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m \
            "$(getent passwd ${USER_UID} | cut -d: -f1)"; \
    fi \
 && apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/* \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# The workspace lives outside the mounted source so that build artefacts never
# land on the host: they are architecture- and path-specific, and mixing a
# host build with a container build is a reliable way to waste an afternoon.
# Creating the directories here also fixes their ownership, which the named
# volumes inherit on first use.
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log \
 && chown -R ${USER_UID}:${USER_GID} /ws

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /ws

# Interactive shells opened with `docker compose exec` do not run the
# entrypoint, so the same setup has to live in the shell profile too.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo 'export GZ_SIM_RESOURCE_PATH=/ws/install/field_robot/share/field_robot/models:${GZ_SIM_RESOURCE_PATH}' >> /home/${USERNAME}/.bashrc \
 && echo "alias cb='cd /ws && colcon build --symlink-install && source install/setup.bash'" >> /home/${USERNAME}/.bashrc \
 && echo 'cd /ws' >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
