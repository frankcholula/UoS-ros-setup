FROM arm64v8/ros:melodic-ros-base-bionic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    ros-melodic-turtlesim \
    ros-melodic-rqt \
    ros-melodic-tf \
    ros-melodic-rviz \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
