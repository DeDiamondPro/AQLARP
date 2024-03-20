from math import *
from .utils import project_to_circle, ease, clamp
from .leg_positions import LegPositions

# The class for the crawling giat
# In this giat one leg is lifted at a time
class CrawlingGiat():
    def __init__(self):
        # Initialize the x value
        self.x = 0

    def get_leg_positions(self, moving_x, moving_z, rotation_heading):
        # Calculate the speed of the robot
        # This is the maximum of the speed in the x and z direction and the rotation speed
        speed = max(sqrt(moving_x * moving_x + moving_z * moving_z), abs(rotation_heading))
        # Get the heading by projecting the x and z values to a circle
        heading_x, heading_z = project_to_circle(moving_x, moving_z, False)

        # Intiliaze an empty leg positions value
        positions = LegPositions()

        # If the robot isn't moving, set all legs to the default position and return
        if speed == 0:
            for leg in range(4):
                positions.legs[leg] = [0, 15, 0]
            return positions

        # Update the x value based on the speed
        self.x += 0.03 * speed
        if self.x >= 4:
            self.x = 0
        # Loop over each leg
        for leg in range(4):
            # Calculate the lifted leg
            # 0-1: Front left lifted
            if self.x < 1:
                lifted_leg = 0
            # 1-2: Back right lifted
            elif self.x < 2:
                lifted_leg = 3
            # 2-3: Front right lifted
            elif self.x < 3:
                lifted_leg = 1    
            # 3-4: Back left lifted
            else:
                lifted_leg = 2
            
            # Adjust the x value based on the current leg
            # The leg should always be lifted if this adjust value is between 0 and 1
            adjusted_x = self.x
            if leg == 1:
                adjusted_x -= 2
            elif leg == 2:
                adjusted_x -= 3
            elif leg == 3:
                adjusted_x -= 1
            if adjusted_x < 0:
                adjusted_x += 4

            # Calculate the leg start and end position
            leg_start_x = 2 * abs(heading_x)
            leg_end_x = -4 * abs(heading_x)
            # If the heading is negative, swap the start and end position
            if heading_x < 0:
                leg_start_x, leg_end_x = leg_end_x, leg_start_x
            
            # Calculate the leg start and end position for the z direction
            leg_start_z = 2 * abs(heading_z)
            leg_end_z = -2 * abs(heading_z)
            # If the heading is negative, swap the start and end position
            if heading_z < 0:
                leg_start_z, leg_end_z = leg_end_z, leg_start_z
            
            # Apply the rotation
            # The direction of this is different for the front and back legs
            if leg < 2:
                leg_start_z += 2 * rotation_heading
                leg_end_z -= 2 * rotation_heading
            else:
                leg_start_z -= 2 * rotation_heading
                leg_end_z += 2 * rotation_heading
            # Limit the z values to -2 and 2
            leg_start_z = clamp(leg_start_z, -2, 2)
            leg_end_z = clamp(leg_end_z, -2, 2)
                
            # Calculate the offset for the legs
            x_offset = leg_start_x + (leg_end_x - leg_start_x) * ((adjusted_x - 1.0) / 3.0)
            y_offset = 15
            z_offset = leg_start_z + (leg_end_z - leg_start_z) * ((adjusted_x - 1.0) / 3.0)

            # If the current leg is the lifted leg, adjust the x, y and z values
            # So they move in the oppisite direction of the movement
            if leg == lifted_leg:
                x_offset = leg_end_x + (leg_start_x - leg_end_x) * ease(adjusted_x)
                y_offset -= 5 * ease(adjusted_x * 2)
                z_offset = leg_end_z + (leg_start_z - leg_end_z) * ease(adjusted_x)

            # Calculate the smoothing value, this is used to make the robot move to the side
            # To help with balance
            raised_leg_x = self.x - floor(self.x)
            ease_point = 0.2
            if raised_leg_x < ease_point:
                smoothing = ease(raised_leg_x / ease_point)
            elif 1 - ease_point < raised_leg_x <= 1:
                smoothing = ease(1 + (raised_leg_x - 1 + ease_point) / ease_point)
            else:
                smoothing = 1

            # make the robot move to the side to help with balance
            x_offset += (-1.3 if lifted_leg < 2 else 1.3) * smoothing
            z_offset += (-1.3 if lifted_leg == 1 or lifted_leg == 2 else 1.3) * smoothing

            # Set the leg position
            positions.legs[leg] = [x_offset, y_offset, z_offset]
        
        # Return the leg positions
        return positions
