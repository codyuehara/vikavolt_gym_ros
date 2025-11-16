FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       rviz
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

#Check out ROS-TCP-Endpoint, ROS2 version
RUN mkdir -p sim_ws/src/ros_tcp_endpoint
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint /sim_ws/src/ros_tcp_endpoint -b ROS2v0.7.0

# f1tenth gym
RUN git clone https://github.com/codyuehara/vikavolt_gym /tmp/vikavolt_gym
RUN cd /tmp/vikavolt_gym && \
    pip3 install -e .

# ros2 gym bridge
RUN mkdir -p sim_ws/src/vikavolt_gym_ros
COPY . /sim_ws/src/vikavolt_gym_ros
RUN source /opt/ros/humble/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

COPY .bashrc /sim_ws/.bashrc
WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
