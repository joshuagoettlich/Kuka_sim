cmake_minimum_required(VERSION 3.8)
project(iiwa_with_robotique)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()