FROM rwthika/ros2:jazzy
ARG DEBIAN_FRONTEND=noninteractive

COPY test_publisher /ws/src/test_publisher
RUN apt-get update && rosdep update && rosdep install -y --from-paths /ws/src --ignore-src --rosdistro jazzy
RUN apt-get install tshark -y
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
