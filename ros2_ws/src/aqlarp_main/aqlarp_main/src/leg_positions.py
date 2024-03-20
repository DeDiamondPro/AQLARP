default = [0], [15], [0]

# Class to store the positions for each of the legs, and the transformations of the body
class LegPositions:
    def __init__(self, legs = [default] * 4, pitch = 0, roll = 0, rotation = 0):
        self.legs = legs
        self.pitch = pitch
        self.roll = roll
        self.rotation = rotation