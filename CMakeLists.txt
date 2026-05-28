cmake_minimum_required(VERSION 3.8)
project(phi_p3dx_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# ---------- C++ executable nodes ----------
add_executable(mapping_example_cpp src/mapping_example.cpp src/mapping_node.cpp)
ament_target_dependencies(mapping_example_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(mapping_log_odds src/mapping_log_odds.cpp src/mapping_node.cpp)
ament_target_dependencies(mapping_log_odds
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(exploration_example_cpp src/exploration_example.cpp src/exploration_node.cpp)
ament_target_dependencies(exploration_example_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(exploring_harmonic_potential_fields src/exploring_harmonic_potential_fields.cpp src/exploration_node.cpp)
ament_target_dependencies(exploring_harmonic_potential_fields
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS mapping_example_cpp exploration_example_cpp mapping_log_odds exploring_harmonic_potential_fields
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts rviz
        DESTINATION share/${PROJECT_NAME})

# ---------- Python Node script ----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/mapping_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mapping_example_py
)

install(PROGRAMS
  scripts/exploration_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME exploration_example_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
