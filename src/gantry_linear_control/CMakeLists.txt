cmake_minimum_required(VERSION 3.8)
project(gantry_linear_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate msg/action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GantryState.msg"
  "action/MoveLinear.action"
  DEPENDENCIES std_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Allow downstream use of generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# Install the Python module package (folder: gantry_linear_control/)
# IMPORTANT: Call this ONCE and do NOT add_subdirectory() the module folder.
# ament_python_install_package(${PROJECT_NAME})

# Install launch files if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
