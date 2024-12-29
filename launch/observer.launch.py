from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='ext_observer_ros',
        executable='ext_observer_node',
        parameters=[{
            # model
            "urdf": "",
            "joint_names": ["joint1", "joint2"],
            "friction": [1.0, 2.0],  # coefficients

            # observer (momentum, disturbance, filter, sm)
            "observer_type": "momentum",
            # momentum observer parameters
            "momentum_gains": [30.0, 30.0],
            # disturbance observer parameters
            "disturbance_sigma": 21.0,
            "disturbance_xeta": 18.0,
            "disturbance_beta": 50.0,
            # filtered dynamics observer
            "filter_cutoff": 8.0,
            "filter_sample": 0.01,
            # sliding mode observer
            "sm_s1": [15.0, 20.0],
            "sm_s2": [10.0, 10.0],
        }],
        remappings=[
            ("/in/joint_state", "/robot/state"),
            ("/out/ext_torque", "/robot/external")
        ]
    )
    return LaunchDescription([
        node
    ])
