from math import *

# Function to clamp a value between a minimum and maximum value
def clamp(value, min, max):
    return max if value > max else min if value < min else value

# Function to ease a value between 0 and 1
# This uses the easeInOutSine easing, for more info see https://easings.net/#easeInOutSine
def ease(x):
    return -(cos(pi * x) - 1) / 2

# Function to project a point to a circle with a diameter of 1
def project_to_circle(x, y, allow_inside = True):
    # Calculate the distance from the origin
    distance = sqrt(x*x + y*y)
    # If the point is already in the circle, return the point
    if distance <= 1 and (allow_inside or distance == 0):
        return x, y
    # Project the point onto the circle
    return x / distance, y / distance
