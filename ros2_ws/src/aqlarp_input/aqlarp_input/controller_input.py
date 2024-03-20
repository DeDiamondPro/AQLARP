import rclpy
import pygame
from rclpy.node import Node
from aqlarp_interfaces.msg import Heading
from std_msgs.msg import Bool

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_input')

        # Initialize pygame and the joysticks
        pygame.init()
        for i in range(pygame.joystick.get_count()):
            pygame.joystick.Joystick(i).init()
        self.get_logger().info('Controller input node started')

        # Initialize variables
        self.enabled = False

        # Create publishers
        self.power_publisher = self.create_publisher(Bool, 'power', 10)
        self.heading_publisher = self.create_publisher(Heading, 'heading', 10)

        # Create a timer to update the heading 100 times per second
        self.timer = self.create_timer(0.01, self.update)

    # Function to apply a dead zone to the controller input, 
    # This prevents the robot from moving when the joystick is not being touched due to light stick drift
    def apply_dead_zone(self, state):
        if abs(state) < 0.2:
            return 0.0
        return state

    # Function to update the heading of the robot
    def update(self):
        try:
            # Process events
            for event in pygame.event.get():
                self.process_event(event)

            # If there are no joysticks, return
            if pygame.joystick.get_count() < 1:
                return
            
            # Initialize the heading message
            heading = Heading()

            # Set the heading values
            heading.x_heading = -self.apply_dead_zone(pygame.joystick.Joystick(0).get_axis(1))
            heading.z_heading = self.apply_dead_zone(pygame.joystick.Joystick(0).get_axis(0))
            heading.rotation_heading = -self.apply_dead_zone(pygame.joystick.Joystick(0).get_axis(3))

            # Log the heading values to the debug log
            self.get_logger().debug(f"Heading: x={heading.x_heading}, z={heading.z_heading}, rotation={heading.rotation_heading}")
            # Publish the heading message
            self.heading_publisher.publish(heading)
        except:
            pass
        
    # Process a controller event
    def process_event(self, event):
        # If the PS button is pressed, toggle the enabled state
        if event.type == pygame.JOYBUTTONDOWN and event.button == 10:
            self.enabled = not self.enabled
            self.power_publisher.publish(Bool(data=self.enabled))
        # If a controller is disconnected, make sure the robot stops moving
        elif event.type == pygame.JOYDEVICEREMOVED:
           self.heading_publisher.publish(Heading())

def main(args = None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create the node
    node = PS4ControllerNode()
    # Run the node
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()