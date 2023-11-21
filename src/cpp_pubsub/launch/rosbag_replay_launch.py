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


       # Launch argument that sets the ros2 bag recording path
       DeclareLaunchArgument(
           "bag_file_path", 
            default_value='rosbag/talker',
            description='Determines the location to save the bag file.'
        ),

        # Launch ros2 bag player
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file_path')],
            output='screen'
        ),

        # Launch onelistener node
        Node(
            package="cpp_pubsub",
            executable="onelistener",
            name="onelistener"
        )
    ])
