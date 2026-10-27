#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ackermann转向几何可视化
在终端实时显示左右轮的转向角度差异
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class AckermannVisualizer:
    def __init__(self):
        rospy.init_node('ackermann_visualizer', anonymous=True)
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        rospy.Subscriber('/racecar/joint_states', JointState, self.joint_callback)
        
        self.cmd_angular_z = 0.0
        self.left_steering = 0.0
        self.right_steering = 0.0
        
        print("\n" + "="*100)
        print(" "*30 + "Ackermann转向几何实时监控")
        print("="*100)
        print("格式: CMD命令 | 左轮角度 | 右轮角度 | 转角差 | 转向状态 | Ackermann验证")
        print("-"*100)
        
        rospy.Timer(rospy.Duration(0.3), self.visualize)
        
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
    
    def visualize(self, event):
        """可视化显示"""
        left_deg = self.rad_to_deg(self.left_steering)
        right_deg = self.rad_to_deg(self.right_steering)
        cmd_deg = self.rad_to_deg(self.cmd_angular_z)
        
        # 计算转角差
        angle_diff = abs(left_deg) - abs(right_deg)
        
        # 判断转向方向和验证Ackermann
        if abs(self.cmd_angular_z) < 0.05:
            direction = "直行 ↑"
            arrow = self.draw_arrow(0)
            ackermann_status = "N/A"
        elif self.cmd_angular_z > 0.05:
            direction = "左转 ↰"
            arrow = self.draw_arrow(1)
            # 检查外八(符号相反)
            if self.left_steering > 0 and self.right_steering < 0:
                ackermann_status = "✗✗ 外八！(符号反)"
            # 左转时,左轮(内侧)应该比右轮(外侧)转角更大
            elif abs(self.left_steering) > abs(self.right_steering):
                ackermann_status = "✓ 正确 (内>外)"
            else:
                ackermann_status = "✗ 错误 (内≤外)"
        else:
            direction = "右转 ↱"
            arrow = self.draw_arrow(-1)
            # 检查外八(符号相反)
            if self.left_steering > 0 and self.right_steering < 0:
                ackermann_status = "✗✗ 外八！(符号反)"
            # 右转时,右轮(内侧)应该比左轮(外侧)转角更大
            elif abs(self.right_steering) > abs(self.left_steering):
                ackermann_status = "✓ 正确 (内>外)"
            else:
                ackermann_status = "✗ 错误 (内≤外)"
        
        # 打印状态行
        print(f"CMD:{cmd_deg:+6.2f}° | "
              f"L:{left_deg:+6.2f}° | "
              f"R:{right_deg:+6.2f}° | "
              f"Δ:{angle_diff:+6.2f}° | "
              f"{direction:8s} {arrow:20s} | "
              f"{ackermann_status:20s}")
    
    def draw_arrow(self, direction):
        """绘制转向箭头"""
        if direction == 0:
            return "    ↑↑"
        elif direction > 0:
            return "  ↖ ↑  (左轮转角大)"
        else:
            return "  ↑ ↗  (右轮转角大)"
    
    @staticmethod
    def rad_to_deg(rad):
        return rad * 180.0 / math.pi
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        viz = AckermannVisualizer()
        viz.spin()
    except rospy.ROSInterruptException:
        pass
