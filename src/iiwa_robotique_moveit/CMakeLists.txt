cmake_minimum_required(VERSION 3.22)
project(iiwa_robotique_moveit)

find_package(ament_cmake REQUIRED)

# This command parses the package.xml file and sets up the package for installation.
ament_package()

# Install launch files, configuration files, RViz configurations, and Gazebo worlds.
# This single command handles all the necessary directories for your package.
install(
    DIRECTORY
        launch
        config
        worlds
    DESTINATION share/${PROJECT_NAME}
    # This pattern excludes the MoveIt Setup Assistant launch file from being installed,
    # as it's typically not needed for running the robot.
    PATTERN "setup_assistant.launch" EXCLUDE
)

# Install the hidden MoveIt Setup Assistant configuration file.
install(
    FILES .setup_assistant
    DESTINATION share/${PROJECT_NAME}
)
