#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
监控Ackermann差速控制
显示左右轮的速度和转向角度差异
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class DifferentialSpeedMonitor:
    def __init__(self):
        rospy.init_node('diff_speed_monitor', anonymous=True)
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        rospy.Subscriber('/racecar/joint_states', JointState, self.joint_callback)
        
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        
        self.left_steering = 0.0
        self.right_steering = 0.0
        self.left_rear_vel = 0.0
        self.right_rear_vel = 0.0
        self.left_front_vel = 0.0
        self.right_front_vel = 0.0
        
        print("\n" + "="*120)
        print(" "*40 + "Ackermann差速控制实时监控")
        print("="*120)
        print("格式: CMD | 转向角(左/右) | 后轮速度(左/右) | 速度差 | 转向状态 | 差速验证")
        print("-"*120)
        
        rospy.Timer(rospy.Duration(0.3), self.monitor)
        
    def cmd_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def joint_callback(self, msg):
        try:
            # 转向角度
            left_steer_idx = msg.name.index('left_steering_joint')
            right_steer_idx = msg.name.index('right_steering_joint')
            self.left_steering = msg.position[left_steer_idx]
            self.right_steering = msg.position[right_steer_idx]
            
            # 轮速
            left_rear_idx = msg.name.index('left_rear_axle')
            right_rear_idx = msg.name.index('right_rear_axle')
            left_front_idx = msg.name.index('left_front_axle')
            right_front_idx = msg.name.index('right_front_axle')
            
            self.left_rear_vel = msg.velocity[left_rear_idx]
            self.right_rear_vel = msg.velocity[right_rear_idx]
            self.left_front_vel = msg.velocity[left_front_idx]
            self.right_front_vel = msg.velocity[right_front_idx]
        except (ValueError, IndexError):
            pass
    
    def monitor(self, event):
        """监控显示"""
        # 转向角度
        left_steer_deg = self.rad_to_deg(self.left_steering)
        right_steer_deg = self.rad_to_deg(self.right_steering)
        angle_diff = abs(left_steer_deg) - abs(right_steer_deg)
        
        # 轮速差异
        avg_left_vel = (self.left_rear_vel + self.left_front_vel) / 2.0
        avg_right_vel = (self.right_rear_vel + self.right_front_vel) / 2.0
        speed_diff = avg_left_vel - avg_right_vel
        speed_ratio = avg_left_vel / avg_right_vel if abs(avg_right_vel) > 0.01 else 1.0
        
        # 判断状态
        if abs(self.cmd_angular) < 0.05:
            direction = "直行 ↑"
            expected_diff = "左右速度应相同"
            if abs(speed_diff) < 0.5:
                diff_status = "✓ 正确"
            else:
                diff_status = f"✗ 速度差过大({speed_diff:.2f})"
        elif self.cmd_angular > 0.05:
            direction = "左转 ↰"
            expected_diff = "右轮(外侧)应更快"
            if avg_right_vel > avg_left_vel + 0.1:
                diff_status = "✓ 正确 (右>左)"
            else:
                diff_status = "✗ 差速不足"
        else:
            direction = "右转 ↱"
            expected_diff = "左轮(外侧)应更快"
            if avg_left_vel > avg_right_vel + 0.1:
                diff_status = "✓ 正确 (左>右)"
            else:
                diff_status = "✗ 差速不足"
        
        # 打印状态行
        print(f"CMD:{self.cmd_angular:+6.2f}° | "
              f"转向 L:{left_steer_deg:+6.2f}° R:{right_steer_deg:+6.2f}° (Δ{angle_diff:+5.2f}°) | "
              f"速度 L:{avg_left_vel:+6.2f} R:{avg_right_vel:+6.2f} | "
              f"速差:{speed_diff:+6.2f} (比例:{speed_ratio:.3f}) | "
              f"{direction:8s} | "
              f"{diff_status:25s}")
    
    @staticmethod
    def rad_to_deg(rad):
        return rad * 180.0 / math.pi
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = DifferentialSpeedMonitor()
        monitor.spin()
    except rospy.ROSInterruptException:
        pass
