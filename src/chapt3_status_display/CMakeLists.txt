cmake_minimum_required(VERSION 3.8)
project(chapt3_status_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chapt3_status_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets) # 查找文件Qt5::Widgets

# 2. 添加可执行文件
add_executable(sys_status_display src/sys_status_display.cpp) # 可执行文件

# 3. 链接依赖
target_link_libraries(sys_status_display Qt5::Widgets) # 链接Qt5库:Qt5不是ros库
ament_target_dependencies(sys_status_display rclcpp chapt3_status_interfaces) # 链接ROS库

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. 安装
install(TARGETS sys_status_display
  DESTINATION lib/${PROJECT_NAME})

ament_package()
