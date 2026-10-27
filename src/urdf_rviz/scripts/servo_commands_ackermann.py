#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import math

class ServoCommands:
    def __init__(self):
        rospy.init_node('servo_commands', anonymous=True)
        
        # Initialize publishers
        self.pub_left_rear_wheel_v = rospy.Publisher('left_rear_wheel_controller/command', Float64, queue_size=1)
        self.pub_right_rear_wheel_v = rospy.Publisher('right_rear_wheel_controller/command', Float64, queue_size=1)
        self.pub_left_front_steer_angle = rospy.Publisher('left_steer_controller/command', Float64, queue_size=1)
        self.pub_right_front_steer_angle = rospy.Publisher('right_steer_controller/command', Float64, queue_size=1)
        
        # Initialize variables
        self.V = 0.0
        self.phi = 0.0
        self.L = 0.7
        self.W = 0.6
        self.min_turn_radius=0.01
        
        # Subscriber
        rospy.Subscriber("cmd_vel", AckermannDriveStamped, self.set_steer)
        
    def set_steer(self, data):
        # Update the current velocity and steering angle
        self.V = data.drive.speed / 0.15
        self.phi = data.drive.steering_angle
        
    def run(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            # Calculate steering angles and wheel velocities
            if abs(self.phi) > 0.001:  # Avoid division by zero for straight driving
                phi_l = math.atan(2 * self.L * math.sin(self.phi) / 
                                 (2 * self.L * math.cos(self.phi) - self.W * math.sin(self.phi)))
                
                phi_r = math.atan(2 * self.L * math.sin(self.phi) / 
                                 (2 * self.L * math.cos(self.phi) + self.W * math.sin(self.phi)))
                # 修正2：正确的速度分配
                R = self.L / math.tan(self.phi)
                sign = 1 if R >= 0 else -1
                R = sign * max(abs(R), self.min_turn_radius)
                
                V_L = self.V * (R - self.W/2) / R
                V_R = self.V * (R + self.W/2) / R
            else:
                phi_l = 0.0
                phi_r = 0.0
                V_L = self.V
                V_R = self.V
            
            # Publish commands
            self.pub_left_rear_wheel_v.publish(V_L)
            self.pub_right_rear_wheel_v.publish(V_R)
            self.pub_left_front_steer_angle.publish(phi_l)
            self.pub_right_front_steer_angle.publish(phi_r)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ServoCommands()
        node.run()
    except rospy.ROSInterruptException:
        pass