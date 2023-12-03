cmake_minimum_required(VERSION 3.5)
project(map_loader)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lrs_interfaces REQUIRED)

include_directories(include)

add_executable(drone_controll_node src/DroneControl.cpp)
ament_target_dependencies(drone_controll_node rclcpp geometry_msgs mavros_msgs lrs_interfaces)

add_executable(mission_loader_node src/MissionLoader.cpp)
ament_target_dependencies(mission_loader_node rclcpp lrs_interfaces)

install(TARGETS
  drone_controll_node
  mission_loader_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
