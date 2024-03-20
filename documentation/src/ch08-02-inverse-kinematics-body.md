# Inverse kinematic of the body
The inverse kinematics of the body is to transform pitch, roll and rotation into offsets for the x, y and z coordinate of a leg. 
## Definitions
\\(l\\) length of the body

\\(w\\) width of the body

\\(pitch\\) desired pitch

\\(roll\\) desired roll

\\(rotation\\) desired rotation

\\(rotation\\) desired rotation

\\(m_{x}\\) leg movement x

\\(m_{y}\\) leg movement y

\\(m_{z}\\) leg movement z

\\(x\\) x position of a leg relative to the center

\\(z\\) z position of a leg relative to the center
## Pitch
The calculations for pitch are as follows:
$$m_{x} = \frac{l(1-\cos(pitch))}{2}$$
$$m_{y} = \frac{l\sin(pitch)}{2}$$
## Roll
The calculations for roll are as follows:
$$m_{z} = \frac{w(1-\cos(roll))}{2}$$
$$m_{y} = \frac{w\sin(roll)}{2}$$
## Rotation
The calculations for rotation are based on a [rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix). These calculations are as follows:

$$m_{x} = (x \times \cos(rotation) - z \times \sin(rotation)) - x$$
$$m_{z} = (x \times \sin(rotation) + z \times \cos(rotation)) - z$$
## Combining Calculations
To combine these calculations you just calculate each one separately and then calculate the sum of all x calculations, then the sum of all y calculations and finally the sum of all z calculations.
## Code
After transforming these calculations into code, we get this:
```py
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
        height_diff_pitch - height_diff_roll,
        movement_z_roll + movement_z_rotation
    )
```