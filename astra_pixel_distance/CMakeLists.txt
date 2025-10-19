cmake_minimum_required(VERSION 3.5)
project(astra_pixel_distance)
find_package(ament_cmake REQUIRED)
install(PROGRAMS
  src/pixel_distance_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
ament_package()