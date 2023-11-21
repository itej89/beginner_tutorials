from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    # Create launch description
    return LaunchDescription([

        # Declare the publisher rate as launch argument
        DeclareLaunchArgument(
            "pub_rate",
            default_value="500",
            description="Publisher rate"
        ),

        # Declare the logging level as launch argument
        DeclareLaunchArgument(
            "log_level",
            default_value=TextSubstitution(text=str("info")),
            description="Logging level"
        ),

       # Launch argument that enables or disables ros2 bag recording
       DeclareLaunchArgument(
           "is_record_bag", 
            default_value='false',
            description='Determines if ros bag record should be enabled.'
        ),

        # Launch ros2 bag recorder
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'rosbag/talker_demo'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('is_record_bag'))
        ),

        # Launch oneservice node
        Node(
            package="cpp_pubsub",
            executable="oneservice",
            name="oneservice",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Launch onetalker node
        Node(
            package="cpp_pubsub",
            executable="onetalker",
            name="onetalker",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[{"pub_rate": LaunchConfiguration("pub_rate")}]
        ),

        # Launch onelistener node
        Node(
            package="cpp_pubsub",
            executable="onelistener",
            name="onelistener",
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
