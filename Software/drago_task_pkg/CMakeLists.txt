cmake_minimum_required(VERSION 3.8)
project(drago_task_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_visualization REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(task_example src/task_example.cpp)

# ── Ect.? ───────────────────────────────────────────────────────────────────
target_include_directories(task_example
  PRIVATE ${moveit_task_constructor_core_INCLUDE_DIRS})

target_link_libraries(task_example
  rclcpp::rclcpp
  moveit_task_constructor_core::moveit_task_constructor_core
  moveit_task_constructor_core::moveit_task_constructor_core_stages
) 

# ── Install ───────────────────────────────────────────────────────────────────
install(
TARGETS task_example
DESTINATION lib/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
