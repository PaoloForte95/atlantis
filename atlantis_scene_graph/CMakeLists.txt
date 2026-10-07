cmake_minimum_required(VERSION 3.8)
project(atlantis_scene_graph)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(standard_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(location_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(scene_graph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(atlantis_util REQUIRED)
find_package(atlantis_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
  standard_msgs
  location_msgs
  geometry_msgs
  scene_graph_msgs
  atlantis_util
  atlantis_core
  Eigen3
)


include_directories(
  include
)



set(executable_name scene_graph_generator)
set(library_name ${executable_name}_core)


add_library(${library_name} SHARED
  src/scene_graph_generator_node.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)




add_library(atlantis_agent_scene_graph_generator SHARED
  src/scene_graph_generators/agent_scene_graph_generator.cpp
)

ament_target_dependencies(atlantis_agent_scene_graph_generator
  ${dependencies}
)
target_link_libraries(atlantis_agent_scene_graph_generator 
  Eigen3::Eigen
)


add_executable(${executable_name}
  src/main.cpp
)  

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "atlantis_scene_graph::SceneGraphGenerator")




install(TARGETS ${library_name} atlantis_agent_scene_graph_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(atlantis_scene_graph scene_graph_generator_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} atlantis_agent_scene_graph_generator)
ament_export_dependencies(${dependencies})

ament_package()
