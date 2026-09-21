cmake_minimum_required(VERSION 3.24)
project(kiss_matcher_localization)

# Keep the standalone package as fast as the original ROS package unless
# the caller explicitly chooses a different build configuration.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationResult.msg"
  "srv/Relocalize.srv"
  "srv/AcceptCandidate.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# The installed KISS Matcher export refers to ROBIN and TBB targets.
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(robin CONFIG REQUIRED)
find_package(kiss_matcher CONFIG REQUIRED)
find_package(small_gicp CONFIG REQUIRED)

add_executable(inter_frame_alignment
  src/inter_frame_alignment.cpp
  src/registration.cpp
)
target_include_directories(inter_frame_alignment PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(inter_frame_alignment
  rclcpp sensor_msgs geometry_msgs tf2_ros pcl_conversions)
target_link_libraries(inter_frame_alignment
  kiss_matcher::kiss_matcher_core
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
)

add_executable(navigation_localizer
  src/navigation_localizer.cpp
  src/registration.cpp
)
target_include_directories(navigation_localizer PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(navigation_localizer
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros pcl_conversions)
target_link_libraries(navigation_localizer
  kiss_matcher::kiss_matcher_core
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigation_localizer "${cpp_typesupport_target}")

add_executable(pcd_pointcloud_publisher src/pcd_pointcloud_publisher.cpp)
target_include_directories(pcd_pointcloud_publisher PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pcd_pointcloud_publisher
  rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcd_pointcloud_publisher ${PCL_LIBRARIES})

install(TARGETS inter_frame_alignment navigation_localizer pcd_pointcloud_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps
  FILES_MATCHING PATTERN "*.pcd")

ament_package()
