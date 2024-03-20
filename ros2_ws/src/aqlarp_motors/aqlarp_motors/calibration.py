import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
import curses
import math
import yaml
import os

def save_offsets(offsets):
    # Find the offsets file
    user_dir = os.path.expanduser("~")
    config_dir = os.path.join(user_dir, '.aqlarp')
    # Create the config directory if it doesn't exist
    os.makedirs(config_dir, exist_ok=True)

    # Save the offsets to the file
    with open(os.path.join(config_dir, 'servo_offsets.yaml'), 'w+', encoding='utf8') as outfile:
        yaml.dump(offsets, outfile)

def load_offsets():
    # Find the offsets file
    user_dir = os.path.expanduser("~")
    offsets_file = os.path.join(user_dir, '.aqlarp/servo_offsets.yaml')

    # Load the offsets file if it exists
    if os.path.exists(offsets_file):
        with open(offsets_file, 'r', encoding='utf8') as infile:
            offsets = yaml.safe_load(infile)
            return offsets
    else:
        return [0] * 12

class ServoCalibrationNode(Node):
    def __init__(self):
        super().__init__('servo_calibration_node')
        # Initialize variables
        self.kit = ServoKit(channels=16)
        self.current_joint = 0
        self.offsets = load_offsets()
        self.joint_to_pin = [6, 7, 8, 9, 10, 11, 3, 4, 5, 0, 1, 2]

    # Clamp a value between a minimum and maximum
    def clamp(self, value, min, max):
        return max if value > max else min if value < min else value

    def calibrate(self, stdscr):
        # Set up curses
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(100)

        # Set the initial servo angles
        for i in range(12):
            self.kit.servo[i].angle = 90 + self.offsets[i]

        while True:
            # Clear the screen
            stdscr.clear()
            # Print the current joint and offset to the screen
            leg_name = ('top ' if self.current_joint < 6 else 'bottom ') + ('left' if math.floor(self.current_joint / 3) % 2 == 0 else 'right')
            joint_name = ('top' if self.current_joint % 3 == 0 else 'bottom' if self.current_joint % 3 == 1 else 'side')
            stdscr.addstr(0, 0, f'Calibrating {leg_name} leg, {joint_name} joint. Press esc or q to exit and save.')
            stdscr.addstr(1, 0, f'Current offset: < {self.offsets[self.joint_to_pin[self.current_joint]]} >')

            # Get the key press
            c = stdscr.getch()
            # Get the pin number of the current joint
            pin = self.joint_to_pin[self.current_joint]
                
            # If escape or q is pressed, exit and save   
            if c == 27 or c == 113:
                for i in range(12):
                    self.kit.servo[i].angle = None
                save_offsets(self.offsets)
                print('Calibration complete, settings saved.')
                break
            # If the arrow keys are pressed, change the current joint or offset
            if c == curses.KEY_UP and self.current_joint < 11:
                self.current_joint += 1
            elif c == curses.KEY_DOWN and self.current_joint > 0:
                self.current_joint -= 1
            elif c == curses.KEY_RIGHT:
                self.offsets[pin] = self.clamp(self.offsets[pin] + 1, -90, 90)
            elif c == curses.KEY_LEFT:
                self.offsets[pin] = self.clamp(self.offsets[pin] - 1, -90, 90)

            # Set the servo angle
            self.kit.servo[pin].angle = 90 + self.offsets[pin]

def main(args=None):
    # Initialize the rclpy
    rclpy.init(args=args)
    # Create the node
    node = ServoCalibrationNode()

    # Run the calibration function
    curses.wrapper(node.calibrate)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
