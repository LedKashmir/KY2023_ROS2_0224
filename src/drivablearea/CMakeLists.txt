cmake_minimum_required(VERSION 3.5)
project(drivablearea)
 
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

#==========================================
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(image_transport REQUIRED)
find_package(novatel_oem7_msgs)
find_package(rosidl_default_generators REQUIRED)  
#=========================================


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 
  ament_lint_auto_find_test_dependencies()
endif()

#add_executable(ros2pcl_test_sub 
# src/subscription_pcl.cpp 
#)
#ament_target_dependencies(ros2pcl_test_sub 
#  rclcpp 
#  sensor_msgs
#  PCL
#)
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries(ros2pcl_test_sub ${PCL_LIBRARIES})
#====================================================================

#ament_auto_add_library(${PROJECT_NAME} SHARED
#  include
#)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(${PROJECT_NAME} PRIVATE
include
${ament_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS} 
${OpenCV_INCLUDE_DIRS}
${Boost_INCLUDE_DIR})

add_executable(drivable_area_node src/drivable_area.cpp  src/drivable_area_tools.cpp src/gpshelper.cpp)

ament_target_dependencies(drivable_area_node 
  rclcpp 
  sensor_msgs
  PCL
  cv_bridge
  rclcpp_lifecycle
  rclcpp_components
  rosidl_default_generators
  std_msgs
  geometry_msgs
  message_filters
  image_transport
  novatel_oem7_msgs
)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(drivable_area_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#=====================================================================

install(TARGETS
  drivable_area_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
 
