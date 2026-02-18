cmake_minimum_required(VERSION 3.8)
project(atlantis_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(dependencies
  Eigen3
  ament_index_cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(atlantis_util SHARED
  src/file_handler.cpp
)
ament_target_dependencies(atlantis_util ${dependencies})


install(
  TARGETS 
  atlantis_util
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(atlantis_util)
ament_package()
