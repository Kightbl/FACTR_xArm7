from setuptools import find_packages, setup

package_name = 'factr_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Jingzhou Liu and Yulong Li',
    maintainer_email='liujason@cmu.edu',
    description='FACTR low-cost force-feedback teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'factr_teleop_franka = factr_teleop.factr_teleop_franka_zmq:main',
            'factr_teleop_xArm7 = factr_teleop.factr_teleop_xArm7:main',
            'factr_teleop_grav_comp_demo = factr_teleop.factr_teleop_grav_comp_demo:main',
            'factr_teleop_grav_comp_demo_xArm7 = factr_teleop.factr_teleop_grav_comp_demo_xArm7:main',
            'torque_slider = factr_teleop.torque_slider:main',

            'factr_teleop_xarm7 = factr_teleop.factr_teleop_xarm7:main',
            'follower_sim = factr_teleop.follower_sim:main',
            'joint_slider_gui = factr_teleop.joint_slider_gui:main',
            'factr_teleop_xarm7_ros = factr_teleop.factr_teleop_xarm7_ros:main',
        ],
    },
)
