cmake_minimum_required(VERSION 3.5)
project(robosense)
 
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
find_package(novatel_oem7_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(image_transport REQUIRED)


find_package(rosidl_default_generators REQUIRED)     


rosidl_generate_interfaces(${PROJECT_NAME}
#"msg/INSPVA.msg"
#"msg/InertialSolutionStatus.msg"
#"msg/Oem7Header.msg"
"msg/Point.msg"
DEPENDENCIES std_msgs
)    
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


link_directories(${PROJECT_NAME} PRIVATE
include
${ament_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS} 
${OpenCV_INCLUDE_DIRS}
${Boost_INCLUDE_DIR})



add_executable(getPtCloud src/getPtCloud.cpp)

rosidl_target_interfaces(getPtCloud ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(getPtCloud 
  rclcpp 
  sensor_msgs
  PCL
  cv_bridge
  rclcpp_lifecycle
  rclcpp_components
  rosidl_default_generators
  std_msgs
  novatel_oem7_msgs
  geometry_msgs
  message_filters
  image_transport
)





add_definitions(${PCL_DEFINITIONS})

target_link_libraries(getPtCloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#=====================================================================

install(TARGETS
  getPtCloud 
  DESTINATION lib/${PROJECT_NAME})
ament_package()
 
