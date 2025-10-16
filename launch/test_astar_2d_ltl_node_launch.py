#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_broadcast_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_broadcast_node",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    test_astar_2d_ltl_node = Node(
        package="erl_path_planning_ros",
        executable="test_astar_2d_ltl_node",
        name="test_astar_2d_ltl_node",
        parameters=[
            {
                "automaton_path": PathJoinSubstitution(
                    [FindPackageShare("erl_path_planning"), "data", "automaton.aut"]
                ),
                "label_map_image_path": PathJoinSubstitution(
                    [FindPackageShare("erl_path_planning"), "data", "label_map.png"]
                ),
            }
        ],
    )
    astar_2d_ltl_node = Node(
        package="erl_path_planning_ros",
        executable="astar_2d_ltl_node",
        name="astar_2d_ltl_node",
        # prefix=["gdbserver localhost:3000"],
        parameters=[
            {
                "default_qos_reliability": "reliable",
                "default_qos_durability": "transient_local",
            }
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("erl_path_planning_ros"), "rviz2", "test_astar_2d_ltl_node.rviz"]),
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            map_broadcast_node,
            test_astar_2d_ltl_node,
            astar_2d_ltl_node,
            rviz_node,
        ]
    )
