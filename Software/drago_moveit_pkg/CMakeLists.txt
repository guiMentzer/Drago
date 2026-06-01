cmake_minimum_required(VERSION 3.22)
project(drago_moveit_pkg)

find_package(ament_cmake     REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(rclcpp_action   REQUIRED)
find_package(control_msgs    REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(calibration_node src/calibration_node.cpp)

ament_target_dependencies(calibration_node rclcpp
                                           rclcpp_action 
                                           sensor_msgs 
                                           control_msgs 
                                           trajectory_msgs
)

install(TARGETS calibration_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()
