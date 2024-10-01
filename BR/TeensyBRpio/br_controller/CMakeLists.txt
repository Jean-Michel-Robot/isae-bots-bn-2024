set (CMAKE_CXX_STANDARD 20)

cmake_minimum_required(VERSION 3.5)
project (br_simu)

add_compile_definitions(_SIMULATION)
add_compile_definitions(_DEBUG)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(br_messages REQUIRED)
# find_package(...) # Add dependencies here

include_directories(include lib)

file(GLOB_RECURSE all_SRCS
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(simulation_br ${all_SRCS})
target_link_libraries(simulation_br m)
ament_target_dependencies(simulation_br rclcpp std_msgs geometry_msgs br_messages)

install(TARGETS
  simulation_br
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()