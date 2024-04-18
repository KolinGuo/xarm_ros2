ARG BASE_IMAGE
FROM ${BASE_IMAGE}

################################################
# SECTION 1: Install xarm_ros2 dependencies    #
################################################
ARG WORKDIR="/workspace"
WORKDIR ${WORKDIR}
# Copy xarm_ros2 ROS package
COPY . src/xarm_ros2

RUN apt-get update \
  && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
  && rm -rf /var/lib/apt/lists/*

# Build xarm_ros2
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --symlink-install \
  && echo "source ${WORKDIR}/install/setup.bash" >> ~/.bashrc


########################################
# SECTION N: Additional config & MISC  #
########################################
# Non-interactive shell
ENV BASH_ENV="/root/.noninteractive_bashrc"
RUN echo "source ${WORKDIR}/install/setup.bash" >> $BASH_ENV

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["/bin/bash"]
