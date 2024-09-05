#!/usr/bin/env python3

# Import necessary ROS and Python libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import tty
import termios

# Define step sizes for linear and angular velocity
LIN_VEL_STEP_SIZE = 2
ANG_VEL_STEP_SIZE = 0.1

# Define a class for the ROS node responsible for keyboard control
class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        # Create publishers for controlling robot components
        self.steering_pub = self.create_publisher(Float64MultiArray, '/mobilearm/fl_wheel_joint_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/mobilearm/fr_wheel_joint_controller/commands', 10)

        # Save the current terminal settings for later restoration
        self.settings = termios.tcgetattr(sys.stdin)

    # Function to read a single keypress from the user
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        # Restore the terminal settings to their original state
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # Function to run the keyboard control loop
    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        # Log an informational message to the console
        self.get_logger().info(self.msg)

        # Initialize variables for controlling the robot
        steer_angle = 0.0
        linear_vel = 0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Check for the Escape key to exit the loop
                    break
                elif key == 'q':  # Check for 'q' to force stop the robot
                    linear_vel = 0.0
                    steer_angle = 0.0
                elif key == 'w':  # Increase forward velocity
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Increase reverse velocity
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Increase right steering angle
                    steer_angle += ANG_VEL_STEP_SIZE
                elif key == 'a':  # Increase left steering angle
                    steer_angle -= ANG_VEL_STEP_SIZE

                # Ensure the steering angle stays within the limits (-1.0 to 1.0)
                if steer_angle > 1.0:
                    steer_angle = 1.0
                if steer_angle < -1.0:
                    steer_angle = -1.0

                print("Steer Angle", steer_angle)
                print("Linear Velocity", linear_vel)

                # Publish control commands to the robot's steering and velocity controllers
                steering_commands = Float64MultiArray(data=[steer_angle])
                velocity_commands = Float64MultiArray(data=[linear_vel])

                self.steering_pub.publish(steering_commands)
                self.velocity_pub.publish(velocity_commands)

# Main function to initialize the ROS node and run the keyboard control loop
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
