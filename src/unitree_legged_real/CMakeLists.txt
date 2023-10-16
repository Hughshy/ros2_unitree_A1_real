cmake_minimum_required(VERSION 3.8)
project(unitree_legged_real)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
# set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(CMAKE_CXX_FLAGS "-O3")
# set(CMAKE_CXX_STANDARD 17)
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/src/unitree_legged_sdk/include
)

link_directories(
    ${CMAKE_SOURCE_DIR}/src/unitree_legged_sdk/lib
    # /home/zcl/myCode/unitree_A1_ctrl/unitree_ros2_to_real-main/unitree_legged_real/lib
    # /home/zcl/myCode/unitree_A1_ctrl/unitree_ros2_to_real-main/unitree_legged_real/unitree_legged_sdk/lib
)
message("-- CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
set(EXTRA_LIBS libunitree_legged_sdk_${ARCH}.so -pthread lcm)


add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(ros2_udp_copy src/ros2_udp_copy.cpp)
target_link_libraries(ros2_udp_copy ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp_copy rclcpp ros2_unitree_legged_msgs sensor_msgs)

# add_executable(ros2_walk_example src/ros2_walk_example.cpp)
# target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
# ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_test src/ros2_test.cpp)
# target_link_libraries(ros2_test -pthread)
ament_target_dependencies(ros2_test rclcpp ros2_unitree_legged_msgs)

install(TARGETS
    ros2_udp          # class structure
    ros2_udp_copy     # origin
    ros2_test
    # ros2_walk_example
    ros2_position_example
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()




