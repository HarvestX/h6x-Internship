cmake_minimum_required(VERSION 3.5)
project(h6x_internship_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

link_directories(${gazebo_dev_LIBRARY_DIRS})

# diff_drive_controller_plugin ======================================
ament_auto_add_library(diff_drive_controller_plugin SHARED
                       src/diff_drive_controller_plugin.cpp)
ament_auto_add_library(gazebo_ros_target_point SHARED
                       src/gazebo_ros_target_point.cpp)
# ===================================================================

# executable (judge) ================================================
ament_auto_add_executable(judge_deviation src/judge/judge_deviation.cpp)
ament_auto_add_executable(game_master src/judge/game_master.cpp)
ament_auto_add_library(judge_goal SHARED src/judge/judge_goal.cpp)
rclcpp_components_register_node(
  judge_goal
    PLUGIN "JudgeGoal"
    EXECUTABLE judge_goal_exec)
# ===================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch models worlds rviz DESTINATION share/${PROJECT_NAME}/)
ament_auto_package()
