cmake_minimum_required(VERSION 3.8)
project(obj_models_small_size_final)

# Find necessary build tools
find_package(ament_cmake REQUIRED)

# This command is the key: it finds and installs all files in the
# 'urdf', 'meshes', and 'launch' directories into the package's 'share' folder.
install(
  DIRECTORY bg bottle box can collision cup cutlery glass remote shoe teapot tube
  DESTINATION share/${PROJECT_NAME}
)

# Install the package.xml file
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()