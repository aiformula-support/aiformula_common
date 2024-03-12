cmake_minimum_required(VERSION 3.5)
project(odometry_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(common_cpp REQUIRED)

file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    tf2_ros
    can_msgs
    nav_msgs
    common_cpp
)
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
