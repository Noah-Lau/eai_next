cmake_minimum_required(VERSION 3.16)
project(eai_bt_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)


set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    std_msgs
    std_srvs
    btcpp_ros2_interfaces
    nav2_msgs
    tf2)

message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
    # 添加 include 目录
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/action
)


######################################################
# Simple example showing how to use and customize the BtExecutionServer
add_executable(eai_bt_controller src/eai_bt_controller.cpp)
ament_target_dependencies(eai_bt_controller ${THIS_PACKAGE_DEPS})

######################################################
# Build an Action Client that calls the sleep action (STATIC version)

add_executable(sleep_client
  src/sleep_action.cpp
  src/sleep_client.cpp)
ament_target_dependencies(sleep_client ${THIS_PACKAGE_DEPS})

######################################################
# Build a client that call the sleep action (Plugin version)

add_library(sleep_plugin SHARED src/sleep_action.cpp)
target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(sleep_plugin ${THIS_PACKAGE_DEPS})

add_library(nav_to_pose_trigger_action_plugin SHARED plugins/action/nav_to_pose_trigger_action.cpp)
target_compile_definitions(nav_to_pose_trigger_action_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(nav_to_pose_trigger_action_plugin ${THIS_PACKAGE_DEPS})

add_executable(sleep_client_dyn src/sleep_client.cpp)
target_compile_definitions(sleep_client_dyn PRIVATE USE_SLEEP_PLUGIN )
target_link_libraries(sleep_client_dyn sleep_plugin )
ament_target_dependencies(sleep_client_dyn ${THIS_PACKAGE_DEPS})

######################################################
# Build Server
add_executable(sleep_server src/sleep_server.cpp)
ament_target_dependencies(sleep_server ${THIS_PACKAGE_DEPS})

######################################################
# Build subscriber_test
add_executable(subscriber_test src/subscriber_test.cpp)
ament_target_dependencies(subscriber_test ${THIS_PACKAGE_DEPS})

######################################################
# the SetBool test
add_executable(bool_client src/bool_client.cpp src/set_bool_node.cpp)
ament_target_dependencies(bool_client ${THIS_PACKAGE_DEPS})

add_executable(bool_server src/bool_server.cpp )
ament_target_dependencies(bool_server ${THIS_PACKAGE_DEPS})

######################################################
# INSTALL

install(TARGETS
  sleep_client
  sleep_client_dyn
  sleep_server
  sleep_plugin
  nav_to_pose_trigger_action_plugin
  subscriber_test
  eai_bt_controller
  bool_client
  bool_server
  DESTINATION lib/${PROJECT_NAME}
  )

######################################################
# INSTALL plugins for other packages to load

install(TARGETS
  sleep_plugin
  nav_to_pose_trigger_action_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

######################################################
# INSTALL Behavior.xml's, ROS config and launch files

install(DIRECTORY
    behavior_trees
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
    )


ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces nav2_behavior_tree)

ament_package()
