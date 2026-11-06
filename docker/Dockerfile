# Development image: ROS 2 Jazzy with this workspace built.
#
#   docker build -f docker/Dockerfile -t so101-control .
#   # simulated bus, no arm needed:
#   docker run --rm -it so101-control ros2 launch so101_bringup so101.launch.py hardware:=fake_bus
#   # real arm on Linux (pass the serial device and your LeRobot calibration):
#   docker run --rm -it --device /dev/ttyACM0 \
#     -v ~/.cache/huggingface/lerobot/calibration:/calibration:ro so101-control \
#     ros2 launch so101_bringup so101.launch.py hardware:=real port:=/dev/ttyACM0 \
#       calibration_file:=/calibration/robots/so_follower/<id>.json
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
WORKDIR /ws
COPY ros2 src/so101-control/ros2

RUN apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN echo 'source /ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
