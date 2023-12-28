cmake_minimum_required(VERSION 3.25.1)

project(xesc_interface)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(include)
ament_package()