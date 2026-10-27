#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试转向方向是否正确
验证: angular.z = -0.6 是否确实让轮子向右转
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time

class SteeringTest:
    def __init__(self):
        rospy.init_node('steering_test', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.joint_sub = rospy.Subscriber('/racecar/joint_states', JointState, self.joint_callback)
        
        self.left_steering_angle = None
        self.right_steering_angle = None
        
        print("\n" + "="*60)
        print("转向方向测试")
        print("="*60)
        
    def joint_callback(self, msg):
        """接收关节状态"""
        try:
            left_idx = msg.name.index('left_steering_joint')
            right_idx = msg.name.index('right_steering_joint')
            self.left_steering_angle = msg.position[left_idx]
            self.right_steering_angle = msg.position[right_idx]
        except ValueError:
            pass
    
    def test_steering(self):
        """测试转向"""
        rospy.sleep(1.0)  # 等待连接建立
        
        print("\n测试1: 发送 angular.z = 0 (直行)")
        print("-" * 60)
        self.send_cmd(0.0, 0.0)
        rospy.sleep(2.0)
        self.print_steering_angles()
        
        print("\n测试2: 发送 angular.z = +0.6 (预期: 左转)")
        print("-" * 60)
        self.send_cmd(0.0, 0.6)
        rospy.sleep(2.0)
        self.print_steering_angles()
        
        print("\n测试3: 发送 angular.z = -0.6 (预期: 右转)")
        print("-" * 60)
        self.send_cmd(0.0, -0.6)
        rospy.sleep(2.0)
        self.print_steering_angles()
        
        print("\n测试4: 回到直行")
        print("-" * 60)
        self.send_cmd(0.0, 0.0)
        rospy.sleep(2.0)
        self.print_steering_angles()
        
        print("\n" + "="*60)
        print("测试完成!")
        print("="*60)
        print("\n总结:")
        print("• 根据ROS右手坐标系:")
        print("  - angular.z > 0 (逆时针) → 应该左转")
        print("  - angular.z < 0 (顺时针) → 应该右转")
        print("• 转向关节角度:")
        print("  - 正值 → 轮子逆时针转(车向左)")
        print("  - 负值 → 轮子顺时针转(车向右)")
        print("\n请在RViz中观察轮子实际转向方向,确认是否与上述一致")
        print("="*60 + "\n")
    
    def send_cmd(self, linear_x, angular_z):
        """发送速度命令"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        print(f"发送: linear.x={linear_x}, angular.z={angular_z}")
    
    def print_steering_angles(self):
        """打印转向角度"""
        if self.left_steering_angle is not None:
            print(f"左转向关节: {self.left_steering_angle:.4f} rad ({self.rad_to_deg(self.left_steering_angle):.2f}°)")
            print(f"右转向关节: {self.right_steering_angle:.4f} rad ({self.rad_to_deg(self.right_steering_angle):.2f}°)")
            
            if abs(self.left_steering_angle) < 0.01:
                print("→ 结果: 直行 ✓")
            elif self.left_steering_angle > 0.01:
                print("→ 结果: 左转 (轮子逆时针)")
            else:
                print("→ 结果: 右转 (轮子顺时针)")
        else:
            print("等待关节状态数据...")
    
    @staticmethod
    def rad_to_deg(rad):
        """弧度转角度"""
        return rad * 180.0 / 3.14159265359

if __name__ == '__main__':
    try:
        tester = SteeringTest()
        tester.test_steering()
    except rospy.ROSInterruptException:
        pass
