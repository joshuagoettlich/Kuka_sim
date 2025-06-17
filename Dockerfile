# ARG specifies the ROS distribution, defaulting to humble
ARG ROS_DISTRO="humble"

# Base Image: ROS 2 Humble
FROM ros:${ROS_DISTRO}

# Set shell to bash and enable sourcing in RUN commands
SHELL ["/bin/bash", "-c"]

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# --- System and ROS Dependency Installation ---

# 1. Set up the Gazebo package repository from osrfoundation.org
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    wget \
    gnupg \
 && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# 2. Update and install all system dependencies in a single layer for efficiency.
# This includes build tools, python libraries, Gazebo, and ROS integration packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libserial-dev \
    libtinyxml2-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-serial \
    python3-vcstool \
    python3-venv \
    xauth \
    xvfb \
    gz-fortress \
    libgz-sim7-dev \
    ros-humble-ros-gz \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
 && rm -rf /var/lib/apt/lists/*

# Fix for potential dpkg/sgml issues
RUN mkdir -p /var/lib/sgml-base && \
    rm -f /var/lib/sgml-base/supercatalog || true && \
    dpkg --configure -a && \
    apt-get install -f -y

# 3. Initialize and update rosdep
RUN rosdep init || true
RUN rosdep update --rosdistro ${ROS_DISTRO}

# --- Build MoveIt2 from Source ---
# This section creates a new workspace for MoveIt, builds it from source,
# and configures the environment to use this build.
RUN apt-get update && apt-get remove -y ros-humble-moveit* && \
    # Create workspace and clone MoveIt tutorials repo
    mkdir -p /root/ws_moveit/src && \
    cd /root/ws_moveit/src && \
    git clone -b ${ROS_DISTRO} https://github.com/moveit/moveit2_tutorials && \
    # Import all necessary MoveIt repositories from the tutorials file
    vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos && \
    # Source the ROS environment and install dependencies
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    # Build the MoveIt workspace using 8 cores
    cd /root/ws_moveit && \
    colcon build --mixin release --parallel-workers 1 && \
    # Clean up apt lists
    rm -rf /var/lib/apt/lists/*

# --- Workspace Setup and Build ---

# Create and set the working directory for the ROS 2 workspace
WORKDIR /root/ros2_ws

# Copy your local source code into the image
# This assumes your Dockerfile is in the root of your project, and your code is in './src'
COPY ./src ./src/src

# Another fix for potential dpkg/sgml issues before final dependency install
RUN mkdir -p /var/lib/sgml-base && \
    rm -f /var/lib/sgml-base/supercatalog || true && \
    dpkg --configure -a && \
    apt-get install -f -y

# Install all dependencies for the copied source code
# Note: This now sources both the base ROS install and our new MoveIt workspace

# Install all dependencies for the copied source code, skipping keys we build from source
RUN apt-get update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /root/ws_moveit/install/setup.bash && \
    rosdep install --from-paths src --ignore-src -y -r --rosdistro ${ROS_DISTRO} --skip-keys "warehouse_ros_mongo mock_components" && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-pinocchio \
    && rm -rf /var/lib/apt/lists/*
    
# Build the entire ROS 2 workspace, skipping specified packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /root/ws_moveit/install/setup.bash && \
    colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip admittance_controller ros2_controllers

# --- Container Configuration and Entrypoint ---

# Add the MoveIt workspace to the .bashrc for automatic sourcing in interactive shells
RUN echo 'source /root/ws_moveit/install/setup.bash' >> /root/.bashrc

# Copy the entrypoint script into the image and make it executable
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set ROS environment variables if needed
ENV ROS_DOMAIN_ID=30
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/ros2_ws/src/src/gazebo_models_worlds_collection/models:${IGN_GAZEBO_RESOURCE_PATH}
ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_ws/src/src/gazebo_ycb/models
ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/root/ros2_ws/install/intel_435/share

# Set the entrypoint for the container
ENTRYPOINT ["/entrypoint.sh"]

# Default command to run when the container starts
CMD ["bash"]