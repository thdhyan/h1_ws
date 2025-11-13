cmake_minimum_required(VERSION 3.5)
project(h1_description)

# ROS 2 uses ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

# Install URDF, meshes, and launch files
install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

