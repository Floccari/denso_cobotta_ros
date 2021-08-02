cmake_minimum_required(VERSION 3.5)
project(denso_cobotta_interfaces)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(denso_cobotta_interfaces
    msg/RobotState.msg
    msg/SafeState.msg
    
    srv/SetMotorState.srv
    srv/GetMotorState.srv
    srv/ClearError.srv
    srv/ClearRobotError.srv
    srv/ClearSafeState.srv
    srv/SetBrakeState.srv
    srv/GetBrakeState.srv
    srv/SetLEDState.srv
    srv/ExecCalset.srv
    
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
