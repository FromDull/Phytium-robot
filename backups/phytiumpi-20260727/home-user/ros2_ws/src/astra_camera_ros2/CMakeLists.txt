cmake_minimum_required(VERSION 3.8)
project(astra_camera_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(OPENNI_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(OPENNI_LIBRARY "${OPENNI_VENDOR_DIR}/lib/libOpenNI2.so")

if(NOT EXISTS "${OPENNI_VENDOR_DIR}/include/OpenNI.h")
  message(FATAL_ERROR "OpenNI2 headers are missing from vendor/include")
endif()
if(NOT EXISTS "${OPENNI_LIBRARY}")
  message(FATAL_ERROR "libOpenNI2.so is missing from vendor/lib")
endif()

add_executable(depth_camera_node src/depth_camera_node.cpp)
target_include_directories(depth_camera_node PRIVATE "${OPENNI_VENDOR_DIR}/include")
target_link_libraries(depth_camera_node "${OPENNI_LIBRARY}")
ament_target_dependencies(depth_camera_node rclcpp sensor_msgs)
set_target_properties(depth_camera_node PROPERTIES INSTALL_RPATH "$ORIGIN")

add_executable(color_camera_node src/color_camera_node.cpp)
ament_target_dependencies(color_camera_node rclcpp sensor_msgs)

add_executable(depth_registration_node src/depth_registration_node.cpp)
ament_target_dependencies(depth_registration_node rclcpp sensor_msgs)

install(TARGETS depth_camera_node color_camera_node depth_registration_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES "${OPENNI_LIBRARY}" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY vendor/OpenNI2 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/camera_snapshot.py scripts/camera_capture_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()
