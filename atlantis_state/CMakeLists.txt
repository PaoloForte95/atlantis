cmake_minimum_required(VERSION 3.8)
project(atlantis_state)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(standard_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  standard_msgs
  pluginlib
)


include_directories(
  include
)


set(executable_name state_generator)
set(library_name ${executable_name}_core)


add_library(${library_name} SHARED
  src/state_generator_node.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)


add_library(atlantis_pddl_state_generator SHARED
  src/state_generators/pddl_state_generator.cpp
)

ament_target_dependencies(atlantis_pddl_state_generator
  ${dependencies}
)


add_executable(${executable_name}
  src/main.cpp
)  

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "atlantis_state::StateGenerator")


install(TARGETS ${library_name} atlantis_pddl_state_generator
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


pluginlib_export_plugin_description_file(atlantis_state pddl_state_generator_plugin.xml)


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
ament_export_libraries(${library_name} atlantis_pddl_state_generator)
ament_export_dependencies(${dependencies})

ament_package()
