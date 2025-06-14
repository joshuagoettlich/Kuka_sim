# 1. Use a modern CMake version required by ROS 2
cmake_minimum_required(VERSION 3.8)

# 2. Project name remains the same
project(gazebo_ycb)

# 3. Find ament_cmake instead of catkin. Add other 'find_package' calls
#    for any other ROS 2 dependencies your package has.
find_package(ament_cmake REQUIRED)
# find_package(gazebo_ros REQUIRED) # Example dependency

# 4. Install directories to the package's share directory.
#    The variable ${CATKIN_PACKAGE_SHARE_DESTINATION} is replaced with
#    the standard 'share/${PROJECT_NAME}'.
install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

# 5. Install environment hooks to the correct 'env-hooks' sub-directory.
#    The ament build system will automatically source them from here.
install(
    DIRECTORY env-hooks/
    DESTINATION share/${PROJECT_NAME}/env-hooks
)

# 6. The ament_package() function replaces catkin_package(). It handles
#    exporting all necessary information for other packages to use.
ament_package()