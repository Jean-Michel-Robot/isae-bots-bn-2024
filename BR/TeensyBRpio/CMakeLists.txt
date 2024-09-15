cmake_minimum_required(VERSION 3.5)

project (br_simu)

add_compile_definitions(__SIMU__)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include lib)

file(GLOB all_SRCS
    "${PROJECT_SOURCE_DIR}/platform/linux/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

^ Just add whatever is in those folders.

add_executable(MyTarget ${all_SRCS})
