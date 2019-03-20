cmake_minimum_required(VERSION 3.5)
project(rplidar_ros)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set(RPLIDAR_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/sdk)

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

FILE(GLOB rplidar_sdk_INCLUDE_DIRS 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/"
  "${RPLIDAR_SDK_PATH}/src/hal/"
  "${RPLIDAR_SDK_PATH}/src/"
)


include_directories(${RPLIDAR_SDK_PATH}/src/)
include_directories(${RPLIDAR_SDK_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)


add_executable(rplidarNode src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})

add_executable(rplidarNodeClient src/client.cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_console REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)





set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${ros2_console_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${rplidar_sdk_INCLUDE_DIRS} ${std_srvs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES}
  ${ros2_console_LIBRARIES} ${sensor_msgs_LIBRARIES} ${std_srvs_LIBRARIES})
target_link_libraries(rplidarNode ${LIBRARY_DIRS})
target_link_libraries(rplidarNodeClient ${LIBRARY_DIRS})
set(LIBS ${ament_cmake_LIBRARIES} ${rclcpp_LIBRARIES} ${ros2_console_LIBRARIES}
  ${sensor_msgs_LIBRARIES})


  #[[install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION bin)]]

  install(TARGETS
  rplidarNode
  rplidarNodeClient
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch rviz sdk
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(ros2_console)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
