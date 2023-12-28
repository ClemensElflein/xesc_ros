cmake_minimum_required(VERSION 3.25.1)
project(xesc_driver)

set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(xesc_interface REQUIRED)
find_package(xesc_2040_driver REQUIRED)
find_package(vesc_driver REQUIRED)

###########
## Build ##
###########


# node executable
add_executable(xesc_driver_node src/xesc_driver.cpp src/xesc_driver_node.cpp)
ament_target_dependencies(xesc_driver_node
        rclcpp
        std_msgs
        xesc_msgs
        serial
        xesc_interface
        xesc_2040_driver
        vesc_driver
)
target_include_directories(
        xesc_driver_node
        PUBLIC
        include
)


install(TARGETS xesc_driver_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()