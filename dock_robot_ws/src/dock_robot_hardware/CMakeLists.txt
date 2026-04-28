cmake_minimum_required(VERSION 3.8)
project(dock_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 获取库架构（如 x86_64-linux-gnu, aarch64-linux-gnu）
message(STATUS "Library architecture: ${CMAKE_LIBRARY_ARCHITECTURE}")
if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "x86_64")
    message(STATUS "Building for x86_64")
    set(CAN_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/x86_64")
elseif(CMAKE_LIBRARY_ARCHITECTURE MATCHES "aarch64")
    message(STATUS "Building for ARM64")
    set(CAN_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/arm_64")
endif()
set(CAN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/include")
if(NOT EXISTS ${CAN_LIBRARY})
    message(WARNING "CAN library not found at ${CAN_LIBRARY}")
endif()
if(NOT EXISTS ${CAN_INCLUDE_DIR}/controlcan.h)
    message(WARNING "controlcan.h not found at ${CAN_INCLUDE_DIR}")
endif()


# 编译生成共享库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/arm_hardware_interface.cpp
  src/can_driver.cpp
)

# 包含头文件目录
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CAN_INCLUDE_DIR}  # 添加CAN头文件路径
)

# 链接CAN库
target_link_libraries(
  ${PROJECT_NAME}
  ${CAN_LIBRARY}/libcontrolcan.so  # 链接CAN库
)

# 包含头文件目录
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

# 链接依赖库
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# 导出插件描述文件（让ROS2能识别插件）
pluginlib_export_plugin_description_file(
  hardware_interface
  arm_hardware_interface.xml
)

# 安装库文件
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

if(CMAKE_LIBRARY_ARCHITECTURE MATCHES "x86_64")
install(
  DIRECTORY thirdparty/lib/x86_64/
  DESTINATION lib
)
elseif(CMAKE_LIBRARY_ARCHITECTURE MATCHES "aarch64")
install(
  DIRECTORY thirdparty/lib/arm_64/
  DESTINATION lib
)
endif()


# 导出项目
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
