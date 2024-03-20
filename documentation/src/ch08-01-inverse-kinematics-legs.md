# Inverse kinematics of the legs
In this chapter we will go over the inverse kinematic calculations for an individual leg.
## Definitions
\\(l_{1}\\) Length of the femur (top part of leg)

\\(l_{2}\\) Length of the tibia (bottom part of leg)

\\(x\\) Desired x position of the leg

\\(y\\) Desired y position of the leg

\\(z\\) Desired z position of the leg

\\(A\\) Angle of the femur joint

\\(B\\) Angle of the tibia joint

\\(C\\) Angle of the sideways joint
## Y-position calculation
The following calculation calculates the angle of the femur joint for a given y-position \\(y\\):
$$A =\arccos\left(\frac{y^2+l_{1}^2-l_{2}^2}{2yl_{1}}\right)$$
The following calculation calculates the angle of the tibia joint for a given y-position \\(y\\):
$$B =\arccos\left(\frac{l_{1}^2+l_{2}^2-y^2}{2l_{1}l_{2}}\right)$$
## X-position calculation
The following calculation calculates the angle of the femur joint for a given x-position \\(x\\) and given a y-position \\(y\\):
$$A = \arctan\left( \frac{x}{y} \right)$$
## Z-position calculation
The following calculation calculates the angle of the sideways joint for a given y-position \\(y\\) and a given z-position \\(z\\):
$$C = \arctan\left( \frac{z}{y} \right)$$
## Combing Calculations
To combine these calculations we first do the X-position and Z-position calculation.
$$A_{x} = \arctan\left( \frac{x}{y} \right)$$
$$C_{z} = \arctan\left( \frac{z}{y} \right)$$
First, since our rotation joint is moved inwards in relation to the leg, we must adjust the height to take the height the rotation joint is creating into account. We can do this as follows:
$$y = y \pm \sin(A_{z}) \times 3$$
Then we must adjust our y value to take into account that we will have to extend our leg further if we want to remain at the same height but go further forward or to the side. This is done as follows:
$$y = \frac{y}{\cos(A_{x}) \times \cos(C_{z})}$$
Then we can do our Y-position calculations with this adjusted \\(y\\) value.
$$A_{y} =\arccos\left(\frac{y^2+l_{1}^2-l_{2}^2}{2yl_{1}}\right)$$
$$B_{y} =\arccos\left(\frac{l_{1}^2+l_{2}^2-y^2}{2l_{1}l_{2}}\right)$$
Then finally we can combine them by adding them together.
$$A = A_{x} + A_{y}$$
$$B = B_{y}$$
$$C = C_{y} + C_{z}$$
## Code
After transforming all these calculations into code, we get this:
```py
def calc_angles(leg, x, y, z):
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
```