cmake_minimum_required(VERSION 3.5)

project(mocap_camera_composer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4ros2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    tf2
    tf2_ros
    mocap4ros2_msgs
    geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME}  
src/mocap_camera_composer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  mocap_camera_composer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
