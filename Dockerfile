FROM ros:jazzy

COPY test_publisher /ws/src/test_publisher
RUN apt-get update && rosdep update && rosdep install -y --from-paths /ws/src --ignore-src --rosdistro jazzy
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
