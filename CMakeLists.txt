cmake_minimum_required(VERSION 3.10)
project(gcopter_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

add_executable(gcopter_ros2_planner src/gcopter_ros2_planner.cpp)

target_include_directories(gcopter_ros2_planner PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

ament_target_dependencies(gcopter_ros2_planner
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(gcopter_ros2_planner
  ${OMPL_LIBRARIES}
)

install(TARGETS gcopter_ros2_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
