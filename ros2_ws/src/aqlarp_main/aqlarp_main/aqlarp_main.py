import rclpy
from math import *
from rclpy.node import Node
from aqlarp_interfaces.msg import JointAngles, GyroAngles, Heading
from std_msgs.msg import Empty, Bool
from src.kinematics import calc_angles
from src.crawling import CrawlingGiat
from src.utils import clamp, project_to_circle

# Leg indexes:                          
# - 0: Front Left,      Pin: 9-11
# - 1: Front Right,     Pin: 0-2
# - 2: Back Left,       Pin: 6-8
# - 3: Back Right,      Pin: 3-5
joint_to_pin = [6, 7, 8, 9, 10, 11, 3, 4, 5, 0, 1, 2]

# The class for the main node
class MainNode(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('aqlarp_main')
       
        # Create a list with the supported giats, this is currentelly only the crawling giat
        # But more can be added at a later data.
        self.giats = [CrawlingGiat()]

        # Initialize some default variables
        self.enabled = False
        self.x_heading = self.z_heading = self.rotation_heading = 0
        self.gyro_pitch = self.gyro_roll  = 90.0
        
        # Create the publisher used to set the joint angles
        self.angle_publisher = self.create_publisher(JointAngles, 'joint_angles', 10)
        # Create the publisher to disable the servos
        self.disable_joints_publisher = self.create_publisher(Empty, 'disable_joints', 10)
        # Create a listener for new gyro data
        self.gyro_listener = self.create_subscription(GyroAngles, 'gryo_angles', self.gyro_update, 10)
        # Create a listener to disable or enable power
        self.power_listener = self.create_subscription(Bool, 'power', self.power_update, 10)
        # Create a listener to get the heading of the robot
        self.heading_listener = self.create_subscription(Heading, 'heading', self.heading_update, 10)

        # Run the main function 100 times per second
        self.timer = self.create_timer(0.01, self.update)

    # Function that is called when new gyro values are recivied
    def gyro_update(self, msg):
        # Save the new values
        self.gyro_pitch = msg.pitch
        self.gyro_roll = msg.roll

    # Function that is called when the power state is changed
    def power_update(self, msg):
        # Save the new power state
        self.enabled = msg.data

    # Function that is called when the heading is changed
    def heading_update(self, msg):
        # Limit the heading to a circle with a radius of 1 and save it
        self.x_heading, self.z_heading = project_to_circle(msg.x_heading, msg.z_heading)
        # Limit the rotation between -1 and 1 and save it
        self.rotation_heading = clamp(msg.rotation_heading, -1, 1)

    # Main function that updates all joint angles
    def update(self):
        # If the robot isn't disable, send the message to disable the joints and return
        if not self.enabled:
            self.disable_joints_publisher.publish(Empty())
            return

        # Get the leg positions from the current giat
        positions = self.giats[0].get_leg_positions(self.x_heading, self.z_heading, self.rotation_heading)

        # Initialize a new empty joint angles object
        jointAngles = JointAngles()
 
        # Loop over each leg
        for leg in range(4):
            # Get the positions of the leg
            leg_positions = positions.legs[leg]
            # Calculate the angles for the leg
            angles = calc_angles(leg, leg_positions[0], leg_positions[1], leg_positions[2], positions.pitch, positions.roll, positions.rotation)
            # Add the angles to the joint angles object
            for joint in range(3):
                jointAngles.angles[leg * 3 + joint] = angles[joint]
            
        # Publish the new angles to the joint angles topic
        self.angle_publisher.publish(jointAngles)

# Main function called on start
def main(args = None):
    # Initiliaze the ros library
    rclpy.init(args=args)

    # Create a new instance of the main node
    node = MainNode()
    # Log that the node has started
    node.get_logger().info('AQLARP core node started')
    # Run the node
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
