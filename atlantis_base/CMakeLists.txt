cmake_minimum_required(VERSION 3.8)
project(atlantis_base)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(atlantis_core REQUIRED)
find_package(atlantis_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navigo REQUIRED)
find_package(Boost COMPONENTS program_options serialization filesystem REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED base)
find_package(nav_msgs REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(material_handler_msgs REQUIRED)
find_package(location_msgs REQUIRED)
find_package(atlantis_util REQUIRED)

if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(dependencies
  atlantis_core
  atlantis_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  visualization_msgs
  Boost
  glog
  yaml-cpp
  Eigen3
  nav2_msgs
  rclcpp_action
  nav_msgs
  navigo
  material_handler_msgs
  ament_index_cpp
  location_msgs
  atlantis_util
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# ---- Existing RViz visualization library (kept) ----
add_library(atlantis_base_rviz_visualization SHARED
  src/rviz_visualization.cpp
)
ament_target_dependencies(atlantis_base_rviz_visualization ${dependencies})
target_link_libraries(atlantis_base_rviz_visualization Eigen3::Eigen)

# ---- Existing RViz config generator (kept) ----
add_library(rviz_config_generator SHARED
  src/rviz_config_generator.cpp)
ament_target_dependencies(rviz_config_generator rclcpp rclcpp_lifecycle yaml-cpp)
target_link_libraries(rviz_config_generator ${YAML_CPP_LIBRARIES})

# ---- BaseSimulator library (the new thin subclass of SimulatorCore) ----
add_library(base_simulator SHARED
  src/base_simulator.cpp
)
ament_target_dependencies(base_simulator ${dependencies})
target_include_directories(base_simulator SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_link_libraries(base_simulator
  atlantis_base_rviz_visualization
  Boost::program_options
  ${EXTERNAL_LIBS}
  ${YAML_CPP_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${GRAPHICSMAGICKCPP_LIBRARIES}
)

# ---- Plugin library ----
add_library(atlantis_base_plugins SHARED
  src/plugins/move_action_plugin.cpp
  src/plugins/load_action_plugin.cpp
  src/plugins/dump_action_plugin.cpp
  src/plugins/material_amount_service_plugin.cpp
  src/plugins/waypoint_list_service_plugin.cpp
)
ament_target_dependencies(atlantis_base_plugins ${dependencies})
target_link_libraries(atlantis_base_plugins
  base_simulator
)

# ---- Executables ----
add_executable(atlantis_base_simulator
  src/main.cpp
)
target_link_libraries(atlantis_base_simulator base_simulator)

add_executable(rviz_config_generator_node
  src/rviz_config_generator_node.cpp
)
target_link_libraries(rviz_config_generator_node rviz_config_generator)

# ---- Tell pluginlib about our plugins ----
pluginlib_export_plugin_description_file(atlantis_core plugins.xml)

#############
## Install ##
#############

install(
  TARGETS
    atlantis_base_rviz_visualization
    base_simulator
    rviz_config_generator
    atlantis_base_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  atlantis_base_simulator
  rviz_config_generator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  atlantis_base_rviz_visualization
  rviz_config_generator
  base_simulator
  atlantis_base_plugins
)
ament_export_dependencies(${dependencies})

ament_package()