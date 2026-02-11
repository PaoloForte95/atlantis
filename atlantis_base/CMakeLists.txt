cmake_minimum_required(VERSION 3.8)
project(atlantis_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(navigo REQUIRED)
find_package(Boost COMPONENTS program_options serialization filesystem REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED base)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(atlantis_util REQUIRED)
find_package(material_handler_msgs REQUIRED)
find_package(location_msgs REQUIRED)

if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


set(dependencies
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
  atlantis_util
  material_handler_msgs
  ament_index_cpp
  location_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(atlantis_base_rviz_visualization SHARED
  src/rviz_visualization.cpp
)

ament_target_dependencies(atlantis_base_rviz_visualization ${dependencies})
target_link_libraries(atlantis_base_rviz_visualization 
  Eigen3::Eigen
)

add_library(rviz_config_generator SHARED
  src/rviz_config_generator.cpp)

ament_target_dependencies(rviz_config_generator rclcpp rclcpp_lifecycle yaml-cpp)

target_link_libraries(rviz_config_generator
   ${YAML_CPP_LIBRARIES}
)


add_library(base_simulator SHARED 
src/base_simulator.cpp)

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

add_executable(atlantis_base_simulator
  src/main.cpp
)

target_link_libraries(atlantis_base_simulator base_simulator)

add_executable(rviz_config_generator_node
  src/rviz_config_generator_node.cpp
)

target_link_libraries(rviz_config_generator_node rviz_config_generator)

#############
## Install ##
#############

install(
  TARGETS 
  atlantis_base_rviz_visualization
  base_simulator
  rviz_config_generator
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
endif()


ament_export_include_directories(include)
ament_export_libraries(atlantis_base_rviz_visualization rviz_config_generator base_simulator)
ament_export_dependencies(${dependencies})

ament_package()
