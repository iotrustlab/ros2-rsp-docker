FROM ros:humble-ros-base

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-irobot-create-description \
    ros-humble-turtlebot4-description \
    ros-humble-irobot-create-control \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create entrypoint script with debug steps
RUN echo '#!/bin/bash\n\
set -ex\n\
\n\
# Source ROS environment\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Debug: Print working directory and list files\n\
pwd\n\
ls -la /urdf/standard/ || echo "Directory not found"\n\
\n\
# Generate URDF from xacro\n\
echo "Generating URDF..."\n\
ros2 run xacro xacro /urdf/standard/turtlebot4.urdf.xacro > /tmp/robot.urdf\n\
\n\
# Debug: Check if URDF was generated and show its size\n\
ls -l /tmp/robot.urdf || echo "URDF not generated"\n\
echo "URDF content:"\n\
cat /tmp/robot.urdf\n\
\n\
# Store URDF content in variable and verify\n\
ROBOT_DESCRIPTION="$(cat /tmp/robot.urdf)"\n\
echo "Description length: ${#ROBOT_DESCRIPTION}"\n\
\n\
# Start robot state publisher with URDF content as parameter\n\
exec ros2 run robot_state_publisher robot_state_publisher \\\n\
    --ros-args \\\n\
    -p robot_description:="${ROBOT_DESCRIPTION}" \\\n\
    --log-level debug' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

# Create directory structure for URDF
RUN mkdir -p /urdf/standard/

ENTRYPOINT ["/ros_entrypoint.sh"]