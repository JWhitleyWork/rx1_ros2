cmake_minimum_required(VERSION 3.5)
project(rx1_gazebo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
  scripts/controller_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller_example
)

install(PROGRAMS
  scripts/controller_gui_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller_gui_example
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    worlds
)
