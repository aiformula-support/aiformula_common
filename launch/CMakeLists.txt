cmake_minimum_required(VERSION 3.5)
project(launch)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/shell
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
