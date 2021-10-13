cmake_minimum_required(VERSION 3.5)
project(denso_cobotta_control)

add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(denso_cobotta_lib REQUIRED)
find_package(denso_cobotta_driver)

ament_export_include_directories(include)
ament_export_libraries(denso_cobotta_control)
ament_export_dependencies(controller_manager hardware_interface rclcpp std_msgs transmission_interface denso_cobotta_lib)


###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/denso_cobotta_control.cpp
  src/denso_cobotta_hw.cpp
)

#add_dependencies(${PROJECT_NAME} denso_cobotta_driver_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_manager
  hardware_interface
  transmission_interface
  std_msgs
  denso_cobotta_lib
  denso_cobotta_driver
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ament_package()
