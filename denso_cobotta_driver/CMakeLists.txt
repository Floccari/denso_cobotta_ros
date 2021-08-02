cmake_minimum_required(VERSION 3.5)
project(denso_cobotta_driver)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(denso_cobotta_lib)
find_package(denso_cobotta_interfaces)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)
ament_export_libraries(denso_cobotta_lib denso_cobotta_interfaces)


###########
## Build ##
###########

include_directories(
  include
# include/${PROJECT_NAME}/cobotta
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/denso_cobotta_driver.cpp
  )

#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  realtime_tools
  std_msgs
  denso_cobotta_lib
  denso_cobotta_interfaces
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
