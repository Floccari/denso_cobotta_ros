cmake_minimum_required(VERSION 3.5)
project(denso_cobotta_gripper)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(denso_cobotta_lib REQUIRED)
find_package(denso_cobotta_driver REQUIRED)

 add_action_files(
   FILES
   GripperMove.action
   VacuumMove.action
 )

#generate_messages(
#   DEPENDENCIES
#   actionlib_msgs
#   std_msgs
#)

ament_export_include_directories(include)
ament_export_libraries(denso_cobotta_gripper)
ament_export_dependencies(controller_manager hardware_interface message_runtime rclcpp std_msgs transmission_interface actionlib actionlib_msgs denso_cobotta_lib)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/denso_cobotta_gripper.cpp
  src/gripper_base.cpp
  src/gripper_parallel.cpp
  src/gripper_vacuum.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
   DESTINATION share/${PROJECT_NAME}
)

ament_package()
