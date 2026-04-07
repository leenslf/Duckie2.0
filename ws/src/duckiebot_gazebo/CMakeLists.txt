cmake_minimum_required(VERSION 3.8)
project(duckiebot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
