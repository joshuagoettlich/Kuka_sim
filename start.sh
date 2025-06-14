#!/bin/bash

# STEP 0: Allow local connections to X server (for GUI applications in Docker)
echo "Allowing local connections to X server..."
xhost +local:root

# CONFIGURATION
# Updated to match the docker-compose.yaml in the Canvas
SERVICE_NAME="ros2_iiwa_gripper"
CONTAINER_NAME="ros2_iiwa_gripper" # Matches container_name in docker-compose.yaml
USE_CONTAINER_NAME=true          # Set to true as container_name is defined
TERMINAL_CMD="gnome-terminal"           # or "xterm", "gnome-terminal", etc.

# STEP 1: Start the container (detached mode)
echo "Starting Docker Compose services in detached mode..."
docker compose up -d

# STEP 2: Wait for container to be up
echo "Waiting a few seconds for the container to initialize..."
sleep 5

# STEP 3: Define the command to open a terminal
EXEC_CMD="docker exec -it ros2_iiwa_gripper bash"
echo "Attempting to open terminals with command: ${EXEC_CMD}"

# STEP 4: Open 4 new terminal windows
# This loop attempts to open a bash shell inside the running container
for i in {1..4}; do
    echo "Opening terminal $i..."
    # This is the line that causes the error if 'konsole' is not installed
    $TERMINAL_CMD -- bash -c "$EXEC_CMD; exec bash" &
    sleep 0.5
done

echo "Attempted to open 4 terminals into the container: ${CONTAINER_NAME}."

# STEP 5: Attach to the container in the current window
echo "Attaching to ${CONTAINER_NAME} in this terminal..."
docker exec -it ${CONTAINER_NAME} bash

echo "Script finished."