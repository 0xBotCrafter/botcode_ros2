cmake_minimum_required(VERSION 3.8)
project(chapt2_3_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1.查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 2.添加可执行文件
add_executable(node_cpp src/node_cpp.cpp)

# 3.链接依赖库
ament_target_dependencies(node_cpp rclcpp)

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

# 4.安装可执行文件
install(TARGETS
  node_cpp
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
