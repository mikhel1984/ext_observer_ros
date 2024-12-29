# ext_observer_ros

ROS2 wrapper for the [ext_observer](https://github.com/mikhel1984/ext_observer) library.
It calculates robot dynamics using *URDF* description and *Orocos* library and publishes observation
of external torques as [JointState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) messages.

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| /in/joint_state | [JointState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) | List of joints with current positions, velocities and troques |
| /out/ext_torque | [JointState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) | Found external torques. Fill only _name_ and _effort_ fields. |

## Parameters

| Name | Type | Description |
|------|------|-------------|
| urdf | string | Path to URDF file. |
| joint_names | string[] | Defines order of numerical values in other arrays. Joint names should be the same as in the URDF file. |
| friction | double[] | Friction coefficients, for multiplication with joint velocities. |
| momentum_gains | double[] | Momentum observer parameters. |
| disturbance_sigma | double | Disturbance observer parameters. |
| disturbance_xeta | double | Disturbance observer parameters. |
| disturbance_beta | double | Disturbance observer parameters. |
| filter_cutoff | double | Filtered dynamics observer parameters. |
| filter_sample | double | Filtered dynamics observer parameters. |
| sm_s1 | double[] | Sliding mode observer parameters. |
| sm_s2 | double[] | Sliding mode observer parameters. |
