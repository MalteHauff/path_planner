cmake_minimum_required(VERSION 3.14)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include(requirements.cmake)
set(CMAKE_CXX_STANDARD 20)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(graph_search_node src/graph_search_node.cpp)

ament_target_dependencies(graph_search_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)




install(TARGETS
  graph_search_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
