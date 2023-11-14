from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "log_level",
            default_value = TextSubstitution(text=str("info")),
            description="Logging level"
        ),

        Node(
            package="cpp_pubsub",
            executable="oneservice",
            name="oneservice",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        Node(
            package="cpp_pubsub",
            executable="onetalker",
            name="onetalker",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        Node(
            package="cpp_pubsub",
            executable="onelistener",
            name="onelistener",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])