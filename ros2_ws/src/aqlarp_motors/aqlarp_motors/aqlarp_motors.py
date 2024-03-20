import rclpy
from rclpy.node import Node
from aqlarp_interfaces.msg import JointAngles
from std_msgs.msg import Empty
from adafruit_servokit import ServoKit
import yaml
import os

pca = ServoKit(channels=16)

# Clamp a value between a minimum and maximum
def clamp(value, min, max):
    return max if value > max else min if value < min else value


def load_offsets():
    # Find the servo offsets file
    user_dir = os.path.expanduser("~")
    offsets_file = os.path.join(user_dir, '.aqlarp/servo_offsets.yaml')
    
    # Load the offsets file if it exists
    if os.path.exists(offsets_file):
        with open(offsets_file, 'r', encoding='utf8') as infile:
            offsets = yaml.safe_load(infile)
            return offsets
    else:
        return [0] * 12


class AngleListener(Node):
    def __init__(self):
        super().__init__('angle_listener')

        # Set update frequency and servo pulse width range
        pca.frequency = 100
        for servo in range(12):
            pca.servo[servo].set_pulse_width_range(500, 2500)

        # Create a subcription on the 'joint_angles' topic
        self.subscription = self.create_subscription(
            JointAngles,
            'joint_angles',
            self.listener_callback,
            10
        )
        # Prevent unused variable message
        self.subscription
        # Load servo offsets
        self.servo_offsets = load_offsets()
        self.get_logger().info(f'Loaded servo offsets: {self.servo_offsets}')

        # Create a subscription on the 'disable_joints' topic
        self.disable_subscription = self.create_subscription(
            Empty,
            'disable_joints',
            self.disable_callback,
            10
        )
        # Prevent unused variable message
        self.disable_subscription

    # Function called when we recieve a message
    def listener_callback(self, msg):
        for i in range(12):
            pca.servo[i].angle = clamp(msg.angles[i] + self.servo_offsets[i], 0, 180)

        # Function called when 'disable_joints' message is received
    def disable_callback(self, msg):
        # Disable all servos
        for i in range(12):
            pca.servo[i].angle = None
            


def main(args = None):
    # Initialze rclpy with arguments if present
    rclpy.init(args=args)

    # Create a listener
    listener = AngleListener()
    listener.get_logger().info('AQLARP motors node started')
    # Start the listener and block this thread until the context is shutdown
    rclpy.spin(listener)

    # Cleanup
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
