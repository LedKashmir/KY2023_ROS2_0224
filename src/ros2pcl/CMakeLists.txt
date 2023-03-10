cmake_minimum_required(VERSION 3.5)
project(ros2pcl)
 
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
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 
  ament_lint_auto_find_test_dependencies()
endif()
 
add_executable(ros2pcl_test_sub 
  src/subscription_pcl.cpp 
)
ament_target_dependencies(ros2pcl_test_sub 
  rclcpp 
  sensor_msgs
  PCL
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(ros2pcl_test_sub ${PCL_LIBRARIES})
install(TARGETS
  ros2pcl_test_sub 
  DESTINATION lib/${PROJECT_NAME})
 
ament_package()
 
