from math import *

leg_length1 = 10.0 # Length of the top part of the leg
leg_length2 = 10.0 # Length of the bottom part of the leg
body_length = 21.1 # Distance between legs on X-axis
body_width = 16.5 # Distance between legs on Z-axis

def calc_angles(leg, x, y, z, pitch, roll, rotation):
    # Calculate the leg offsets for pitch, roll and rotation and adjust the x, y and z values
    offsets = calc_leg_offsets(leg, pitch, roll, rotation)
    x += offsets[0]
    y += offsets[1]
    z += offsets[2]

    # Calculate top joint position to reach X coordinate
    x_pos_a = atan(-x / y)
    # Calculate side joint position to reach Z coordinate
    z_pos_c = atan(z / y)
    # Adjust the target height because of the Z-joint having 3cm offset until the leg
    if leg == 0 or leg == 3:
        y += sin(z_pos_c) * 3
    else:
        y -= sin(z_pos_c) * 3
    # Adjust the target height for the X and Z offsets
    y = y / (cos(x_pos_a) * cos(z_pos_c))
    # Calculate top joint position to reach Y coordinate
    y_pos_a = acos((y * y + leg_length1 * leg_length1 - leg_length2 * leg_length2) / (2 * y * leg_length1))
    # Calculate bottom joint position to reach Y coordinate
    y_pos_b = acos((leg_length1 * leg_length1 + leg_length2 * leg_length2 - y * y) / (2 * leg_length1 * leg_length2))
    # Merge all angles and convert them from radians to degrees
    A = degrees(x_pos_a + y_pos_a)
    B = degrees(y_pos_b)
    C = degrees(z_pos_c)
    # Adjust the angles for the servo orientation and return the result
    return (
        (90 - A) if leg == 0 or leg == 3 else (90 + A),
        B if leg == 0 or leg == 3 else (180 - B), 
        (90 + C) if leg < 2 else (90 - C)
    )

def calc_leg_offsets(leg, pitch, roll, rotation):
    # Get the X and Z coordinate of the leg
    x = (body_length / 2) if leg < 2 else -(body_length / 2)
    z = -(body_width / 2) if leg == 0 or leg == 3 else (body_width / 2)
    # Calculate the height difference of the leg to reach the target pitch
    height_diff_pitch = x * sin(radians(pitch))
    # Calculate the height difference of the leg to reach the target roll
    height_diff_roll = z * sin(radians(roll))
    # Calculate the X and Z offsets to keep the body centered
    movement_x_pitch = x * (1 - cos(radians(pitch)))
    movement_z_roll = z * (1 - cos(radians(roll)))
    # Calculate the rotation offsets
    movement_x_rotation = (x * cos(radians(rotation)) - z * sin(radians(rotation))) - x
    movement_z_rotation = (x * sin(radians(rotation)) + z * cos(radians(rotation))) - z
    # Merge the offsets and return them
    return (
        movement_x_pitch + movement_x_rotation,
        height_diff_pitch + height_diff_roll,
        movement_z_roll + movement_z_rotation
    )
