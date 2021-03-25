FROM ros:noetic
LABEL MAINTAINER=owerko@seas.upenn.edu

RUN apt-get update -y\
    && apt-get install -y python3-catkin-tools python3-osrf-pycommon python3-scipy

WORKDIR /dcist_ws

COPY . ./src/mid

RUN catkin config --init --install --extend /opt/ros/noetic
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin build

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]