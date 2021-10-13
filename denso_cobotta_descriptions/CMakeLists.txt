cmake_minimum_required(VERSION 3.5)
project(denso_cobotta_descriptions)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY cobotta_description DESTINATION share/${PROJECT_NAME})
