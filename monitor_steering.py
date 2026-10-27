#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
实时监控转向角度和cmd_vel命令
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class SteeringMonitor:
    def __init__(self):
        rospy.init_node('steering_monitor', anonymous=True)
        
        # 订阅
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        rospy.Subscriber('/racecar/joint_states', JointState, self.joint_callback)
        
        # 数据
        self.cmd_angular_z = 0.0
        self.left_steering = 0.0
        self.right_steering = 0.0
        
        print("\n" + "="*80)
        print("转向监控器 - 实时显示")
        print("="*80)
        print("格式: cmd_vel.angular.z | 左转向关节 | 右转向关节 | 转向判断")
        print("-"*80)
        
        # 定时打印
        rospy.Timer(rospy.Duration(0.5), self.print_status)
        
    def cmd_callback(self, msg):
        self.cmd_angular_z = msg.angular.z
    
    def joint_callback(self, msg):
        try:
            left_idx = msg.name.index('left_steering_joint')
            right_idx = msg.name.index('right_steering_joint')
            self.left_steering = msg.position[left_idx]
            self.right_steering = msg.position[right_idx]
        except ValueError:
            pass
    
    def print_status(self, event):
        """打印状态"""
        # 判断转向方向
        if abs(self.left_steering) < 0.05:
            direction = "直行"
            arrow = "↑"
        elif self.left_steering > 0.05:
            direction = "左转"
            arrow = "↰"
        else:
            direction = "右转"
            arrow = "↱"
        
        # 检查是否一致
        cmd_direction = ""
        if abs(self.cmd_angular_z) < 0.05:
            cmd_direction = "直行"
        elif self.cmd_angular_z > 0.05:
            cmd_direction = "左转(+)"
        else:
            cmd_direction = "右转(-)"
        
        # 打印
        print(f"CMD: {self.cmd_angular_z:+.3f} [{cmd_direction:8s}] | "
              f"左轮: {self.left_steering:+.3f} | "
              f"右轮: {self.right_steering:+.3f} | "
              f"{arrow} {direction:4s}")
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = SteeringMonitor()
        monitor.spin()
    except rospy.ROSInterruptException:
        pass
