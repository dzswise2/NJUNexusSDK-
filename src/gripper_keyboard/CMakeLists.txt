cmake_minimum_required(VERSION 3.8)
project(gripper_keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(infra_msg REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBEVDEV REQUIRED libevdev)

# qwer_keyboard_node
add_executable(qwer_keyboard_node src/qwer_keyboard_node.cpp)
target_include_directories(qwer_keyboard_node PRIVATE ${LIBEVDEV_INCLUDE_DIRS})
target_link_libraries(qwer_keyboard_node ${LIBEVDEV_LIBRARIES})
ament_target_dependencies(qwer_keyboard_node
  rclcpp
  infra_msg
)

# Install
install(TARGETS
  qwer_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
