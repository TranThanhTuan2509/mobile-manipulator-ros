#!/usr/bin/env python3

import rospy
import sys
import time
import threading
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TeleopController:
    def __init__(self):
        rospy.init_node("free_control", anonymous=True)
        self.pub = rospy.Publisher("/meca/diff_drive_controller/cmd_vel", Twist, queue_size=10)
        self.servo1_pub = rospy.Publisher("/meca/servo1_controller/command", Float64, queue_size=10)
        self.servo2_pub = rospy.Publisher("/meca/servo2_controller/command", Float64, queue_size=10)
        self.twist = Twist()

    def servo_oscillation(self):
        """Continuously moves the servos back and forth while the robot is running."""
        t = 0
        while not rospy.is_shutdown():
            servo1_value = 0.2 + 0.4 * np.sin(t)  # Oscillates between 0.1 and 0.6
            servo2_value = 0.5 + 0.4 * np.cos(t)  # Oscillates between 0.1 and 0.9
            
            self.servo1_pub.publish(servo1_value)
            self.servo2_pub.publish(servo2_value)
            
            t += 0.1
            time.sleep(0.1)

    def run(self, x, z):
        servo_thread = threading.Thread(target=self.servo_oscillation)
        servo_thread.daemon = True
        servo_thread.start()
        
        while not rospy.is_shutdown():
            try:
                user_input = input("\nEnter 'x z' (linear_x angular_z) or 'stop' to stop: ").strip()
                
                if user_input.lower() == "stop":
                    self.twist.linear.x = x
                    self.twist.angular.z = z
                    self.pub.publish(self.twist)
                    print("Robot Stopped!")
                    continue

                x, z = map(float, user_input.split())

                self.twist.linear.x = x
                self.twist.angular.z = z

                self.pub.publish(self.twist)
                rospy.loginfo(f"Moving -> Linear: {x} m/s, Angular: {z} rad/s")

            except ValueError:
                print("Invalid input. Please enter two numbers (x z) or 'stop'.")

if __name__ == "__main__":
    teleop = TeleopController()
    teleop.run(x=0.2, z=0)
