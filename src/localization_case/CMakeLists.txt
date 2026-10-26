cmake_minimum_required(VERSION 3.8)
project(localization_case)

find_package(ament_cmake REQUIRED)

# Launch files, configs and the measurement script are installed as data:
# no Python package lookups are needed (see PR08 assignment).
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/measure_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
