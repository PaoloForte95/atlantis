cmake_minimum_required(VERSION 3.8)
project(atlantis_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(location_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(atlantis_util REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# A shared library holding SimulationWorld and SimulatorCore.
# ActionPlugin is header only.
add_library(${PROJECT_NAME} SHARED
  src/simulation_world.cpp
  src/simulator_core.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  location_msgs
  atlantis_util
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# Install headers so other packages can include them.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Tell downstream packages what we export.
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rclcpp rclcpp_lifecycle std_msgs location_msgs yaml-cpp)

ament_package()