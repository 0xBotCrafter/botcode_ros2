cmake_minimum_required(VERSION 3.8)
project(chapt4_cpp_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖的包
find_package(ament_cmake REQUIRED)
find_package(chapt4_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_control src/turtle_control.cpp) # 可执行文件
add_executable(turtle_patrol src/turtle_patrol.cpp)

ament_target_dependencies(turtle_control chapt4_interfaces rclcpp geometry_msgs turtlesim) # 链接依赖
ament_target_dependencies(turtle_patrol chapt4_interfaces rclcpp)

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

# exe安装
install(TARGETS turtle_control turtle_patrol
  DESTINATION lib/${PROJECT_NAME})

# launch 安装
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
