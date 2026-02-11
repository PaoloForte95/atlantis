cmake_minimum_required(VERSION 3.8)
project(atlantis_discrete_event)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Boost COMPONENTS program_options serialization filesystem REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED base)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(atlantis_util REQUIRED)

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
  atlantis_util
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)


# add_library(discrete_event_simulator SHARED 
# src/discrete_event_simulator.cpp)


# ament_target_dependencies(discrete_event_simulator ${dependencies})
# target_link_libraries(discrete_event_simulator
#    Boost::program_options
#    ${EXTERNAL_LIBS}
#    ${YAML_CPP_LIBRARIES}
#    ${OMPL_LIBRARIES}
# )


# add_executable(atlantis_discrete_event_simulator
#   src/main.cpp
# )

# target_link_libraries(atlantis_discrete_event_simulator discrete_event_simulator)

# #############
# ## Install ##
# #############

# install(
#   TARGETS 
#   discrete_event_simulator
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# install(TARGETS 
# atlantis_discrete_event_simulator
# RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()



# ament_export_include_directories(include)
# ament_export_libraries(discrete_event_simulator)
# ament_export_dependencies(${dependencies})

ament_package()
