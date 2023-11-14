from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Create launch description
    return LaunchDescription([

        # Declare the publisher rate as launch argument
        DeclareLaunchArgument(
            "pub_rate",
            default_value = "500",
            description="Publisher rate"
        ),

        # Declare the logging level as launch argument
        DeclareLaunchArgument(
            "log_level",
            default_value = TextSubstitution(text=str("info")),
            description="Logging level"
        ),

        # Laucnh oneservice node
        Node(
            package="cpp_pubsub",
            executable="oneservice",
            name="oneservice",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Laucnh onetalker node
        Node(
            package="cpp_pubsub",
            executable="onetalker",
            name="onetalker",
            arguments=['--ros-args','--log-level', LaunchConfiguration('log_level')],
            parameters=[
            {"pub_rate": LaunchConfiguration("pub_rate")}]
        ),

        # Laucnh onelistener node
        Node(
            package="cpp_pubsub",
            executable="onelistener",
            name="onelistener",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )

    ])