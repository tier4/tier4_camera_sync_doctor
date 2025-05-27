cmake_minimum_required(VERSION 3.5)
project(tier4_camera_sync_doctor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tier4_camera_sync_doctor.cpp
)

# target_link_libraries(${PROJECT_NAME}
#   yaml-cpp
# )

ament_target_dependencies(${PROJECT_NAME}
  SYSTEM
  sensor_msgs
  builtin_interfaces
  rcl_yaml_param_parser
  rcutils
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tier4_camera_sync_doctor::Tier4CameraSyncDoctor"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
