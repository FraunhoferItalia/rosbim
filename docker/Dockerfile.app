FROM fraunhoferitalia/rosbim:humble

RUN apt-get -y update && apt-get -y install \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-joy

RUN apt-get -y update && apt-get -y install \
    ros-${ROS_DISTRO}-moveit \ 
    ros-${ROS_DISTRO}-moveit-visual-tools  \
    ros-${ROS_DISTRO}-rqt-* \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 

RUN apt-get -y update && apt-get -y install \
    python3-pip && \ 
    pip3 install trimesh==3.20.2 && \
    pip3 install pcg-gazebo==0.7.12 && \
    # Fixes import error with pcg-gazebo https://stackoverflow.com/a/72747002
    pip3 install markupsafe==2.0.1 && \
    pip3 install pytransform3d && \
    pip3 install pygame && \
    pip3 install open3d && \
    apt-get -y install byobu && \
    apt-get -y install clang && \
    apt-get -y install clang-format

# for pcg_gazebo used in rosbim_gazebo_worlds
RUN apt-get -y update && apt-get -y install \
    libspatialindex-dev pybind11-dev libgeos-dev && \
    pip install markupsafe==2.0.1

ENV DEBIAN_FRONTEND=interactive

RUN echo "source ${ROS_WS_DIR}/src/environment.sh" >> ~/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc && \
    echo "alias rbuild='(cd ${ROS_WS_DIR} && colcon build --cmake-args -DBUILD_TESTING=OFF && source install/setup.sh)'" >> ~/.bash_aliases && \
    echo "alias rclean='(cd ${ROS_WS_DIR} && rm -rf install/ log/ build/)'"  >> ~/.bash_aliases

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

