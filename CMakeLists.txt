cmake_minimum_required(VERSION 3.16)
project(perception_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # 必须显式查找
find_package(nav_msgs REQUIRED)      # 如果用到了 Odometry
find_package(perception_hardware REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. 生成参数库 (注意：第一个参数建议写成带后缀的，方便区分)
generate_parameter_library(
  perception_system_controller_params_lib
  src/perception_system_controller_params.yaml
)

# 3. 编译 Controller 库
add_library(${PROJECT_NAME} SHARED
  src/perception_system_controller.cpp
  # 注意：如果下面这些文件里也有 PLUGINLIB_EXPORT_CLASS 且类名冲突，编译会过，但运行会挂。
  # 建议前期只保留主逻辑文件，测试通过后再加。
  src/test_mock_camera_controller.cpp
  src/test_mock_many_cameras_controller.cpp
  src/test_t265_camera_controller.cpp
)

# 4. 【关键】建立显式依赖，确保头文件先生成
add_dependencies(${PROJECT_NAME} perception_system_controller_params_lib)

# 5. 【关键】包含生成头文件的路径
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> # 必须包含二进制生成目录！
  $<INSTALL_INTERFACE:include>
)

# 6. 依赖注入 (不要把参数库放在这里，它是 target)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  sensor_msgs
  geometry_msgs
  nav_msgs
  perception_hardware
  OpenCV
  cv_bridge
)

# 7. 链接参数库 target
target_link_libraries(${PROJECT_NAME} perception_system_controller_params_lib)

# 8. 导出插件描述文件
pluginlib_export_plugin_description_file(
  controller_interface
  perception_controller_plugins.xml
)

# 9. 安装规则 (修正了安装路径)
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