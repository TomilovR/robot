cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CleaningTask.action"
  DEPENDENCIES geometry_msgs
)

# Install Python executables
install(PROGRAMS
  action_cleaning_robot1/cleaning_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_action_server
)

install(PROGRAMS
  action_cleaning_robot1/cleaning_action_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_action_client
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()