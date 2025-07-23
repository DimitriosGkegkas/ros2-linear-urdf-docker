from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = {
        "robot_description": ParameterValue(value=Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
                " ",
                "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 name:=ur ur_type:=ur20"
            ]
        ), value_type=str)
    }
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    

    joint_state_publisher_node = Node(
        package="ur20_display",
        executable="joint_state_publisher",
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur20_display"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    plotter_node = Node(
        package="ur20_display",
        executable="plotter.py",
        name="plotter",
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        plotter_node,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
