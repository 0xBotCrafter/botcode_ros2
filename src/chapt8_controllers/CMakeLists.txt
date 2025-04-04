cmake_minimum_required(VERSION 3.8)
project(chapt8_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

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

include_directories(include)
# ================添加库文件=====================
add_library(chapt8_controllers SHARED 
  src/spin_controller.cpp 
  src/move_controller.cpp)
ament_target_dependencies(chapt8_controllers  pluginlib )
install(TARGETS chapt8_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 导出插件描述文件
pluginlib_export_plugin_description_file(chapt8_controllers chapt8_controllers.xml)
install(DIRECTORY include/
  DESTINATION include/
)

add_executable(test_plugin src/test_plugin.cpp)
ament_target_dependencies(test_plugin pluginlib)
install(TARGETS test_plugin
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
