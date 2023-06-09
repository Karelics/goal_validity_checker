cmake_minimum_required(VERSION 3.8)
project(goal_checker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(goal_checker_msgs REQUIRED)

    

install(DIRECTORY
  launch
  param
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
# add_executable(cpp_executable src/my_cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
install(TARGETS
  # cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  goal_checker/goal_validity_checker.py
  DESTINATION lib/${PROJECT_NAME}
)

set(plugin_dependencies

    rclcpp
    behaviortree_cpp_v3
    nav2_behavior_tree
    goal_checker_msgs
    )
add_library(goal_checker_bt_node SHARED src/bt_srv_node.cpp)

ament_target_dependencies(goal_checker_bt_node ${plugin_dependencies})
target_compile_definitions(goal_checker_bt_node PRIVATE BT_PLUGIN_EXPORT)


install(TARGETS ${library_name}
            goal_checker_bt_node
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
