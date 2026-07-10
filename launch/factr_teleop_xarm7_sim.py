"""
factr_teleop_xarm7_sim.py
-------------------------
Launch file for the FACTR xArm7 simulation test rig.

Starts three nodes:
  1. follower_sim        — receives desired joint positions, publishes reaction torques
  2. joint_slider_gui    — PyQt5 GUI for commanding follower joint positions
  3. factr_teleop_xarm7  — FACTR leader arm controller (reads torques, drives Dynamixels)

Usage (from <repo_root>):
  ros2 launch launch/factr_teleop_xarm7_sim.py

Override parameters at launch time:
  ros2 launch launch/factr_teleop_xarm7_sim.py tracking_kp:=8.0 torque_gain:=2.0

Node graph:
  [joint_slider_gui] --/xarm7/desired_joint_pos--> [follower_sim]
                                                         |
                                              /xarm7/external_joint_torque
                                                         |
                                                         v
                                              [factr_teleop_xarm7]
                                                         |
                                              /xarm7/leader_joint_cmd  (diagnostics)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments (all overridable on the command line) ────────────────

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='xarm7_sim.yaml',
        description='FACTR YAML config file (relative to configs/ directory)'
    )

    tracking_kp_arg = DeclareLaunchArgument(
        'tracking_kp',
        default_value='0.6',
        description='follower_sim: stiffness gain converting position error to torque'
    )

    torque_gain_arg = DeclareLaunchArgument(
        'torque_gain',
        default_value='0.6',
        description='follower_sim: overall output torque scale'
    )

    gravity_enable_arg = DeclareLaunchArgument(
        'gravity_enable',
        default_value='True',
        description='follower_sim: include simulated gravity torque component'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='100.0',
        description='follower_sim: torque publish rate in Hz'
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='xArm7.urdf',
        description='follower_sim: path to xArm7 URDF for Pinocchio gravity model'
    )

    # ── Node 1: follower_sim ──────────────────────────────────────────────────
    # Starts immediately — must be up before the FACTR controller starts
    # subscribing to /xarm7/external_joint_torque.

    follower_sim_node = Node(
        package='factr_teleop',             # adjust if you put it in a separate pkg
        executable='follower_sim',
        name='follower_sim',
        output='screen',
        parameters=[{
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'tracking_kp':     LaunchConfiguration('tracking_kp'),
            'torque_gain':     LaunchConfiguration('torque_gain'),
            'gravity_enable':  LaunchConfiguration('gravity_enable'),
        }],
        remappings=[
            ('/xarm7/desired_joint_pos',      '/xarm7/desired_joint_pos'),
            ('/xarm7/external_joint_torque',  '/xarm7/external_joint_torque'),
        ]
    )

    # ── Node 2: joint_slider_gui ──────────────────────────────────────────────
    # Small delay so follower_sim subscriber is ready before first publish.

    slider_gui_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='factr_teleop',
                executable='joint_slider_gui',
                name='joint_slider_gui',
                output='screen',
                remappings=[
                    ('/xarm7/desired_joint_pos', '/xarm7/desired_joint_pos'),
                ]
            )
        ]
    )

    # ── Node 3: factr_teleop_xarm7 ────────────────────────────────────────────
    # Delayed by 2 s to allow follower_sim and slider GUI to come up first,
    # and to give the Dynamixel driver time to initialise.

    factr_teleop_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='factr_teleop',
                executable='factr_teleop_xarm7',
                name='factr_teleop',
                output='screen',
                parameters=[{
                    'config_file': LaunchConfiguration('config_file'),
                    'simulation': False,
                }],
                remappings=[
                    ('/xarm7/external_joint_torque', '/xarm7/external_joint_torque'),
                    ('/xarm7/leader_joint_cmd',      '/xarm7/leader_joint_cmd'),
                ]
            )
        ]
    )

    # ── Startup message ───────────────────────────────────────────────────────

    startup_msg = LogInfo(msg=(
        '\n'
        '========================================================\n'
        '  FACTR xArm7 Simulation\n'
        '========================================================\n'
        '  Node startup order:\n'
        '    t=0s  follower_sim       (torque publisher)\n'
        '    t=1s  joint_slider_gui   (PyQt5 position commander)\n'
        '    t=2s  factr_teleop_xarm7 (leader arm controller)\n'
        '\n'
        '  Useful topics to monitor:\n'
        '    ros2 topic echo /xarm7/desired_joint_pos\n'
        '    ros2 topic echo /xarm7/external_joint_torque\n'
        '    ros2 topic echo /xarm7/leader_joint_cmd\n'
        '    ros2 topic hz   /xarm7/external_joint_torque\n'
        '========================================================\n'
    ))

    return LaunchDescription([
        # arguments
        config_file_arg,
        tracking_kp_arg,
        torque_gain_arg,
        gravity_enable_arg,
        publish_rate_arg,
        urdf_path_arg,
        # startup info
        startup_msg,

        # nodes (in dependency order)
        follower_sim_node,
        slider_gui_node,
        factr_teleop_node,
    ])
