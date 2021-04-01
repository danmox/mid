FROM ros:noetic
LABEL MAINTAINER=owerko@seas.upenn.edu

RUN apt-get update -y && apt-get install -y python3 python3-pip python3-catkin-tools python3-osrf-pycommon libblas-dev liblapack-dev
#RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 2 && update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 2
RUN pip3 install cvxpy
RUN pip3 install scipy matplotlib

WORKDIR /dcist_ws
COPY . ./src/mid
RUN catkin config --init --extend /opt/ros/noetic
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin build
RUN echo "source /dcist_ws/devel/setup.bash" >> $HOME/.bashrc

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]