#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def ackermann_publisher():
    # Initialize the node
    rospy.init_node('ackermann_publisher', anonymous=True)
    
    # Create a publisher for AckermannDriveStamped messages
    pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    
    # Set the publishing rate (10Hz in this example)
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        # Create a new AckermannDriveStamped message
        msg = AckermannDriveStamped()
        
        # Set the header timestamp
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"  # Set your frame ID
        
        # Set the drive parameters
        msg.drive.speed = 5.0      # Speed 
        msg.drive.steering_angle = 0.3  # Steering angle of 0.3 radians
        
        # Optional: You can set other AckermannDrive fields if needed
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        msg.drive.steering_angle_velocity = 0.0
        
        # Publish the message
        pub.publish(msg)
        
        # Log the published message
        rospy.loginfo("Publishing Ackermann command: speed=%.2f, angle=%.2f", 
                     msg.drive.speed, msg.drive.steering_angle)
        
        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        ackermann_publisher()
    except rospy.ROSInterruptException:
        pass