from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='factr_teleop',
            executable='factr_teleop_xarm7_zmq',
            name='factr_teleop_xarm7_zmq',
            parameters=[{'config_file': 'xarm7_sim.yaml'}]
        )
    ])
