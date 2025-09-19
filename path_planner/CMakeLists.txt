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
find_package(Eigen3 REQUIRED)          

set(CSAPS_SUBMODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/csaps)
set(CSAPS_TARGET "")

if(EXISTS ${CSAPS_SUBMODULE_DIR})
  message(STATUS "csaps submodule found at ${CSAPS_SUBMODULE_DIR}")

  if(EXISTS ${CSAPS_SUBMODULE_DIR}/CMakeLists.txt)
    add_subdirectory(${CSAPS_SUBMODULE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/third_party/csaps_build EXCLUDE_FROM_ALL)
    if(TARGET csaps)
      set(CSAPS_TARGET csaps)
    elseif(TARGET csaps_cpp)
      set(CSAPS_TARGET csaps_cpp)
    endif()
  endif()

  if(NOT CSAPS_TARGET)
    file(GLOB CSAPS_SOURCES RELATIVE ${CSAPS_SUBMODULE_DIR}
      "${CSAPS_SUBMODULE_DIR}/src/*.cpp"
      "${CSAPS_SUBMODULE_DIR}/src/**/*.cpp"
    )

    if(CSAPS_SOURCES)
      set(CSAPS_SOURCES_FULL "")
      foreach(_f ${CSAPS_SOURCES})
        list(APPEND CSAPS_SOURCES_FULL ${CSAPS_SUBMODULE_DIR}/${_f})
      endforeach()
      add_library(csaps_local STATIC ${CSAPS_SOURCES_FULL})

      target_include_directories(csaps_local SYSTEM BEFORE PUBLIC
        /usr/include/eigen3
      )

      target_link_libraries(csaps_local PUBLIC Eigen3::Eigen)

      target_include_directories(csaps_local PUBLIC
        ${CSAPS_SUBMODULE_DIR}/include
        ${CSAPS_SUBMODULE_DIR}
      )

      set(CSAPS_TARGET csaps_local)
    else()
      add_library(csaps_local INTERFACE)
      target_include_directories(csaps_local INTERFACE ${CSAPS_SUBMODULE_DIR}/include ${CSAPS_SUBMODULE_DIR})
      # header-only fallback
      target_link_libraries(csaps_local INTERFACE Eigen3::Eigen)
      set(CSAPS_TARGET csaps_local)
    endif()
  endif()

  if(NOT TARGET "csaps::csaps")
    add_library(csaps::csaps ALIAS ${CSAPS_TARGET})
  endif()
else()
  message(FATAL_ERROR "csaps submodule not found at ${CSAPS_SUBMODULE_DIR}. Run:\n  git submodule update --init --recursive")
endif()

# --- build local adore implementation library (cubic piecewise & friends) ---
# Only collect implementation sources from mad/ and env/ to avoid pulling in node sources
file(GLOB_RECURSE ADORE_IMPL_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/mad/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/env/*.cpp"
)

# debug: list found sources during configure
message(STATUS "ADORE_IMPL_SRCS found: ${ADORE_IMPL_SRCS}")

if(ADORE_IMPL_SRCS)
  add_library(adore_impl STATIC ${ADORE_IMPL_SRCS})

  target_include_directories(adore_impl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mad
    ${CMAKE_CURRENT_SOURCE_DIR}/src/env
  )

  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)

  # try to find system dlib (fallback to manual link if not found)
  find_library(DLIB_LIB NAMES dlib PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

  if(DLIB_LIB)
    message(STATUS "Found dlib at ${DLIB_LIB}")
    target_link_libraries(adore_impl PUBLIC Eigen3::Eigen ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${DLIB_LIB})
  else()
    message(STATUS "dlib not found via find_library; link manually if needed")
    target_link_libraries(adore_impl PUBLIC Eigen3::Eigen ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
  endif()

  set_target_properties(adore_impl PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()


add_executable(graph_search_node src/graph_search_node.cpp)

ament_target_dependencies(graph_search_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# plain form to match ament_target_dependencies (do NOT use PRIVATE/PUBLIC here)
target_link_libraries(graph_search_node
  adore_impl
  ${CSAPS_TARGET}
  Eigen3::Eigen
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
