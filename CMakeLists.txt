cmake_minimum_required(VERSION 3.16)
project(perception_controller)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_hardware REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. 编译 Controller 库
add_library(${PROJECT_NAME} SHARED
  src/test_mock_camera_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/home/test/control_ws/src/perception_hardware/perception_manager/include"
)

# 3. 依赖注入
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  sensor_msgs
  OpenCV
  cv_bridge
)

# 4. 导出插件描述文件 (关键)
pluginlib_export_plugin_description_file(
  controller_interface
  perception_controller_plugins.xml
)

# 5. 安装规则
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  FILES perception_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
