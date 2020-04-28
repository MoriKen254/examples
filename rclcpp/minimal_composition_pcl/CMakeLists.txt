cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_composition_pcl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common)

include_directories(include
    ${PCL_COMMON_INCLUDE_DIRS})

add_library(composition_pcl_nodes SHARED
            src/publisher_pcl_node.cpp
            src/subscriber_pcl_node.cpp)
target_compile_definitions(composition_pcl_nodes
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(composition_pcl_nodes
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs)
target_link_libraries(composition_pcl_nodes
    ${Boost_LIBRARIES})

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

add_executable(composition_publisher_pcl src/standalone_publisher_pcl.cpp)
target_link_libraries(composition_publisher_pcl composition_pcl_nodes)
ament_target_dependencies(composition_publisher_pcl
  rclcpp composition_pcl_nodes)

add_executable(composition_subscriber_pcl src/standalone_subscriber_pcl.cpp)
target_link_libraries(composition_subscriber_pcl composition_pcl_nodes)
ament_target_dependencies(composition_subscriber_pcl
  rclcpp composition_pcl_nodes)

add_executable(composition_composed_pcl src/composed_pcl.cpp)
target_link_libraries(composition_composed_pcl composition_pcl_nodes)
ament_target_dependencies(composition_composed_pcl rclcpp class_loader)

install(TARGETS
  composition_pcl_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition_publisher_pcl
  composition_subscriber_pcl
  composition_composed_pcl
  DESTINATION lib/${PROJECT_NAME})

ament_package()
