cmake_minimum_required(VERSION 3.8)
project(intel_435)

find_package(ament_cmake REQUIRED)

# This installs the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# This installs the urdf/sdf files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# --- THIS IS THE CRITICAL PART ---
# Ensure this block exists to install your 3D models.
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()