cmake_minimum_required(VERSION 3.16)
project(localization_visualizer)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

add_executable(localization_visualizer_node src/localization_visualizer_node.cpp)
target_include_directories(localization_visualizer_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(localization_visualizer_node
  rclcpp sensor_msgs tf2 tf2_ros pcl_conversions)
target_link_libraries(localization_visualizer_node ${PCL_LIBRARIES})

install(TARGETS localization_visualizer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
