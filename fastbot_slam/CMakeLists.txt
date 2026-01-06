cmake_minimum_required(VERSION 3.8)
project(fastbot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED) 
find_package(std_srvs REQUIRED)          
find_package(geometry_msgs REQUIRED)     
find_package(sensor_msgs REQUIRED)       
find_package(nav_msgs REQUIRED)

# Optional: Nav2/Cartographer packages if we ever add C++ nodes using them
# find_package(cartogrpaher-ros REQUIRED)
# find_package(nav2_amcl REQUIRED)
# find_package(nav2_map_server REQUIRED)
# find_package(nav2_lifecycle_manager REQUIRED)
# find_package(nav2_map_server REQUIRED)
# find_package(nav2_behaviors REQUIRED)
# find_package(nav2_bt_navigator REQUIRED)
# find_package(nav2_planner REQUIRED)
# find_package(nav2_controller REQUIRED)

# -----------------------------
# Install directories (launch, config, rviz, maps)
# -----------------------------
install(
  DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Install Python nodes/scripts
# -----------------------------
# Make sure the scripts are executable
install(PROGRAMS
  scripts/timestamp_filter_node.py
  scripts/trigger_global_localization.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
