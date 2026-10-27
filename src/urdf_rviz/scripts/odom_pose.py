#!/usr/bin/env python3
#-*-coding:utf-8-*-
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, Twist, PoseStamped, Transform, TransformStamped
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates

class odom_class:
    def __init__(self):
        self.namespace = rospy.get_param('~namespace', 'aaack8')
        self.last_received_pose = Pose()
        self.last_recieved_stamp = None

        self.frame_name = self.namespace + "/odom"
        self.child_frame_name = self.namespace + "/base_footprint"

        self.pub_pose = rospy.Publisher('pose', PoseStamped, queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        try:
            arrayIndex = msg.name.index(f"{self.namespace}::base_footprint")
        except ValueError:
            pass
        else:
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_recieved_stamp = rospy.Time.now()

    def pub(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.last_recieved_stamp is None:
                rate.sleep()
                continue

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.last_recieved_stamp
            pose_msg.header.frame_id = self.frame_name
            pose_msg.pose = self.last_received_pose

            self.pub_pose.publish(pose_msg)

            tf = TransformStamped(
                header=Header(
                    frame_id=self.frame_name,
                    stamp=self.last_recieved_stamp
                ),
                child_frame_id=self.child_frame_name,
                transform=Transform(
                    translation=self.last_received_pose.position,
                    rotation=self.last_received_pose.orientation
                )
            )
            self.tf_pub.sendTransform(tf)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    try:
        odom = odom_class()
        odom.pub()
    except rospy.ROSInterruptException:
        pass