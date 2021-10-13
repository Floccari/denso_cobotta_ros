import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("denso_cobotta_descriptions"),
            "cobotta_description",
            "cobotta.urdf.xacro",
        ),
        mappings={'gripper_type'  : 'parallel'}
    )
    robot_description = {
        "robot_description": robot_description_config.toxml()
    }

    robot_description_semantic_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("denso_cobotta_moveit_config"),
            "config",
            "cobotta.srdf.xacro",
        )
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config.toxml()
    }

    kinematics_yaml = load_yaml(
        "denso_cobotta_moveit_config", "config/kinematics.yaml"
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory("denso_cobotta_moveit_config"), "launch")
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Warehouse mongodb server
    #mongodb_server_node = Node(
    #    package="warehouse_ros_mongo",
    #    executable="mongo_wrapper_ros.py",
    #    parameters=[
    #        {"warehouse_port": 33829},
    #        {"warehouse_host": "localhost"},
    #        {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
    #    ],
    #    output="screen"
    #)

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            #mongodb_server_node,
        ]
    )
