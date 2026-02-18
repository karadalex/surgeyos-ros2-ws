cmake_minimum_required(VERSION 3.8)
project(robot_description2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
