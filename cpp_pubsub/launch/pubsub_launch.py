from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_pubsub",
            executable="oneservice",
            name="oneservice",
        ),

        Node(
            package="cpp_pubsub",
            executable="onetalker",
            name="onetalker",
        ),

        Node(
            package="cpp_pubsub",
            executable="onelistener",
            name="onelistener",
        ),
    ])