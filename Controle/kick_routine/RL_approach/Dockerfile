# Use an official ROS image as a parent image
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    gazebo9 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install stable-baselines3
RUN pip3 install stable-baselines3[extra]

# Copy the URDF and world files into the container
COPY martha_urdf_description /root/catkin_ws/src/martha_description/
COPY soccer_field.world /root/catkin_ws/src/soccer_world/worlds/
COPY ball.sdf /root/.gazebo/models/soccer_ball/model.sdf
COPY goal.sdf /root/.gazebo/models/goal/model.sdf

# Set up the catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  cd /root/catkin_ws && \
                  catkin_make"

# Source the workspace
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set the entrypoint to start Gazebo with the custom world
ENTRYPOINT ["/bin/bash", "-c", "source /root/.bashrc && roslaunch gazebo_ros empty_world.launch world_name:=/root/catkin_ws/src/soccer_world/worlds/soccer_field.world"]

