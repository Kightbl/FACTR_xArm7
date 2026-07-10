from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='factr_teleop',
            executable='factr_teleop_grav_comp_demo_xArm7',
            name='factr_teleop_grav_comp_demo_xArm7',
            parameters=[{'config_file': 'grav_comp_xArm7.yaml'}]
        )
    ])
