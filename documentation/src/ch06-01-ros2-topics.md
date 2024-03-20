# Ros2 Topics
This chapter outlines the various ros2 topics AQLARP has that you can interact with to adapt the robot to your use case.

## Heading topic
Topic name: `heading`

Package: `aqlarp_main`

Description: with this topic you can specify where the robot should go to and at what speed. The heading parameters will be limited to a circle of radius one and the rotation parameter will be limited between -1 and 1.

Data type: [Heading](https://github.com/DeDiamondPro/AQLARP/blob/master/ros2_ws/src/aqlarp_interfaces/msg/Heading.msg)

Values:
- `float32 x_heading`
- `float32 z_heading`
- `float32 rotation`
## Power topic
Topic name: `power`

Package: `aqlarp_main`

Description: with this topic you can set the power to the servos on or off.

Data type: Boolean

## Gyro topic
Topic name: `gryo_angles`

Package: `aqlarp_sensors`

Description: all gyro related data is published on this topic, this is published 100 times per second.

Data type: [GyroAngles](https://github.com/DeDiamondPro/AQLARP/blob/master/ros2_ws/src/aqlarp_interfaces/msg/GyroAngles.msg)

Values:
- `float32 pitch`
- `float32 roll`
- `float32[3] angular_velocity`
- `float32[3] acceleration`

## Joint angles topic
Topic name: `joint_angles`

Package: `aqlarp_motors`

Description: with this topic you can specify the angles that all servos should be set to.

Data type: [JointAngles](https://github.com/DeDiamondPro/AQLARP/blob/master/ros2_ws/src/aqlarp_interfaces/msg/JointAngles.msg)

Values:
- `float32[12] angles`

## Disable joints topic
Topic name: `disable_joints`

Package: `aqlarp_motors`

Description: power off all servos

Data type: Empty