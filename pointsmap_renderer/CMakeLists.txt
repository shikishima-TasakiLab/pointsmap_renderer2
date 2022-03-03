cmake_minimum_required(VERSION 3.5)
project(pointsmap_renderer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)

find_package(vgm_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(voxelgridmap
  src/voxel_grid_map.cpp
)
ament_target_dependencies(voxelgridmap
  rclcpp
  std_msgs
  sensor_msgs
  tf2_ros
  cv_bridge
  pcl_conversions
  vgm_msgs
)
target_link_libraries(voxelgridmap ${PCL_LIBRARIES})

add_executable(vgm_pub src/vgm_pub.cpp src/vgm_pub_node.cpp)
ament_target_dependencies(vgm_pub
  rclcpp
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  vgm_msgs
)
target_link_libraries(vgm_pub ${PCL_LIBRARIES} voxelgridmap)

add_executable(pointsmap_renderer src/pointsmap_renderer.cpp src/pointsmap_renderer_node.cpp)
ament_target_dependencies(pointsmap_renderer
  rclcpp
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
  vgm_msgs
)
target_link_libraries(pointsmap_renderer ${PCL_LIBRARIES} voxelgridmap)

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)

install(TARGETS
  voxelgridmap vgm_pub pointsmap_renderer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
