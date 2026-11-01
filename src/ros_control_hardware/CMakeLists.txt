cmake_minimum_required(VERSION 3.16)
project(ros_control_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros_control_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/system_hardware.cpp
  # src/driver/transport/can_transport.cpp
  # src/driver/protocol/protocol_factory.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp hardware_interface pluginlib ros_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

pluginlib_export_plugin_description_file(
  hardware_interface ros_control_hardware_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES ros_control_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME})

# ---- 驱动层验收（不需要 CAN，也不需要真电机）----
add_executable(driver_test
  test/driver_test.cpp
  src/driver/transport/loopback_transport.cpp
  src/driver/protocol/fake_protocol.cpp
  src/driver/protocol/encos_protocol.cpp
  src/driver/protocol/yiyou_protocol.cpp
)

target_compile_features(driver_test PRIVATE cxx_std_17)
target_include_directories(driver_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hardware_interface pluginlib ros_control_msgs)

ament_package()
