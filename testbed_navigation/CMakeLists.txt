cmake_minimum_required(VERSION 3.8)
project(testbed_navigation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

# Test tools: ros2 run testbed_navigation <script>
install(
  PROGRAMS
    scripts/drive_test.py
    scripts/localization_test.py
    scripts/navigation_test.py
    scripts/plot_route.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
