cmake_minimum_required(VERSION 3.16)
project(pcd_cloud_publisher)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(pcd_cloud_publisher_node src/pcd_cloud_publisher_node.cpp)
target_include_directories(pcd_cloud_publisher_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pcd_cloud_publisher_node
  rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcd_cloud_publisher_node ${PCL_LIBRARIES})

install(TARGETS pcd_cloud_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY clouds/ DESTINATION share/${PROJECT_NAME}/clouds
  FILES_MATCHING PATTERN "*.pcd")

ament_package()
