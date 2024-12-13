cmake_minimum_required(VERSION 3.14.4)
project(ccom_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(dubins_curves REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(project11_navigation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)


set(dependencies
  behaviortree_cpp
  dubins_curves
  geometry_msgs
  project11_navigation
  rclcpp
  tf2
)

add_library(ccom_planner_bt_plugin SHARED
  src/dubins_astar.cpp
  src/plan_action.cpp
)

target_include_directories(ccom_planner_bt_plugin
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(ccom_planner_bt_plugin ${dependencies})

target_compile_definitions(ccom_planner_bt_plugin PRIVATE  BT_PLUGIN_EXPORT )

install(TARGETS ccom_planner_bt_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(ccom_planner_bt_plugin)
ament_export_dependencies(${dependencies})
ament_package()
