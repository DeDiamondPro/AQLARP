import rclpy
from rclpy.node import Node
from aqlarp_interfaces.msg import GyroAngles
import board
import busio
import adafruit_mpu6050
from math import *

# Function to convert a 2D vector to degrees
def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle

class GyroAccelNode(Node):
    def __init__(self):
        super().__init__('gyro')

        # Create a publisher on the 'gyro_angles' topic
        self.publisher = self.create_publisher(GyroAngles, 'gryo_angles', 10)

        # Initialize the I2C bus and the MPU6050 sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mpu = adafruit_mpu6050.MPU6050(i2c)

        # Create a timer to read the sensor data 100 times per second
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # Read the sensor data
        acceleration = self.mpu.acceleration

        # Initialize the GyroAngles message
        msg = GyroAngles()
        # Set the roll and pitch angles
        msg.roll = vector_2_degrees(acceleration[0], acceleration[2])
        msg.pitch = vector_2_degrees(acceleration[1], acceleration[2])
        # Set the acceleration and angular velocity
        msg.acceleration = acceleration
        msg.angular_velocity = self.mpu.gyro

        # Log the roll, pitch, acceleration, and angular velocity to the debug log
        self.get_logger().debug(f'Roll: {msg.roll}, Pitch: {msg.pitch}, Acceleration: {msg.acceleration}, Angular Velocity: {msg.angular_velocity}', throttle_duration_sec = 0.1)
        # Publish the message
        self.publisher.publish(msg)

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create the node
    gyro_accel_node = GyroAccelNode()
    # Run the node
    rclpy.spin(gyro_accel_node)
    
    # Cleanup
    gyro_accel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()