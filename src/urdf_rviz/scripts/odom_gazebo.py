#!/usr/bin/env python3
#-*-coding:utf-8-*-

import rospy
import tf2_ros
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry



class odom_class:
    def __init__(self):
        
        self.namespace = rospy.get_param('~namespace', 'aaack8')
        
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        
        self.frame_name = self.namespace+"/odom"
        self.child_frame_name = self.namespace+"/base_footprint"
        
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)
        
    def sub_robot_pose_update(self, msg):
        try:
            arrayIndex = msg.name.index(f"{self.namespace}::base_footprint")
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
            self.last_recieved_stamp = rospy.Time.now()
    
    def pub(self):
        
        rate = rospy.Rate(20) 
        while not rospy.is_shutdown():
            if self.last_recieved_stamp is None:
                rate.sleep()
                continue

            cmd = Odometry()
            cmd.header.stamp = self.last_recieved_stamp
            cmd.header.frame_id = self.frame_name
            cmd.child_frame_id = self.child_frame_name
            cmd.pose.pose = self.last_received_pose
            cmd.twist.twist = self.last_received_twist
            cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]

            cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                            0, 1e-3, 1e-9, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e-9]


            self.pub_odom.publish(cmd)

            tf = TransformStamped(
                header=Header(
                    frame_id=cmd.header.frame_id,
                    stamp=cmd.header.stamp
                ),
                child_frame_id=cmd.child_frame_id,
                transform=Transform(
                    translation=cmd.pose.pose.position,
                    rotation=cmd.pose.pose.orientation
                )
            )
            self.tf_pub.sendTransform(tf)
            
            rate.sleep()
        


if __name__=="__main__":
    rospy.init_node("odom_publisher")
    try:
        odom=odom_class()
        odom.pub()
    except rospy.ROSInterruptException:
        pass
    