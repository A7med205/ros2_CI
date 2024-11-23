cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)

add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2)
rosidl_target_interfaces(tortoisebot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(tortoisebot_action_client src/tortoisebot_action_client.cpp)
ament_target_dependencies(tortoisebot_action_client rclcpp rclcpp_action)
rosidl_target_interfaces(tortoisebot_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  tortoisebot_action_server
  tortoisebot_action_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

set(TESTFILES
  test/main.cpp
  test/test_tortoise_action.cpp
)

ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

set_tests_properties(${PROJECT_NAME}_test PROPERTIES
  TIMEOUT 100
)

ament_target_dependencies(${PROJECT_NAME}_test rclcpp rclcpp_action std_msgs geometry_msgs)
rosidl_target_interfaces(${PROJECT_NAME}_test ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  ${PROJECT_NAME}_test
  DESTINATION lib/${PROJECT_NAME}
)

endif()

ament_package()
