from adafruit_servokit import ServoKit

# Initialize the PCA9685 using the default address (0x40).
pca = ServoKit(channels=16)

def main():
    # Loop over all pins
    for i in range(16):
        # Set pulse width range to 500-2500
        pca.servo[i].set_pulse_width_range(500, 2500)
        # Set angle to 90 degrees
        pca.servo[i].angle = 90
    
if __name__ == '__main__':
    main()