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
# ===================================================================

# executable (judge) ================================================
ament_auto_add_executable(judge_courceout src/judge/judge_courseout.cpp)
ament_auto_add_executable(send_log src/judge/send_log.cpp)
ament_auto_add_executable(judge_goal src/judge/judge_goal.cpp)
# ===================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch models worlds rviz DESTINATION share/${PROJECT_NAME}/)
ament_auto_package()
