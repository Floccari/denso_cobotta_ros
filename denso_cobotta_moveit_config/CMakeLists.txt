cmake_minimum_required(VERSION 3.5)
project(denso_cobotta_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share)

ament_package()
