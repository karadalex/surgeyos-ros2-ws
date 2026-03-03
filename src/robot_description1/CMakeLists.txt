cmake_minimum_required(VERSION 3.8)
project(robot_description1)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf meshes rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
