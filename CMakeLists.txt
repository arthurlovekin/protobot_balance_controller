cmake_minimum_required(VERSION 3.8)
project(protobot_balance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Add a compile directive for a shared library
add_library(protobot_balance_controller SHARED
  src/protobot_balance_controller.cpp
)

ament_target_dependencies(protobot_balance_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

#Export for pluginlib description file
pluginlib_export_plugin_description_file(controller_interface protobot_balance_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gmock(test_protobot_balance_controller
    test/test_protobot_balance_controller.cpp
    test/test_load_protobot_balance_controller.cpp
  )

ament_target_dependencies(test_protobot_balance_controller
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/
  DESTINATION include/protobot_balance_controller
)

ament_package()
