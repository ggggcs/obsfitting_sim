#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
验证Ackermann转向几何实现
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class AckermannVerifier:
    def __init__(self):
        rospy.init_node('ackermann_verifier', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.joint_sub = rospy.Subscriber('/racecar/joint_states', JointState, self.joint_callback)
        
        self.left_steering_angle = None
        self.right_steering_angle = None
        
        # 从URDF获取的参数
        self.wheelbase = 0.335  # meters
        self.track_width = 0.365  # meters
        
        print("\n" + "="*80)
        print("Ackermann转向几何验证")
        print("="*80)
        print(f"车辆参数:")
        print(f"  轴距 (wheelbase): {self.wheelbase} m")
        print(f"  轮距 (track_width): {self.track_width} m")
        print("="*80 + "\n")
        
    def joint_callback(self, msg):
        """接收关节状态"""
        try:
            left_idx = msg.name.index('left_steering_joint')
            right_idx = msg.name.index('right_steering_joint')
            self.left_steering_angle = msg.position[left_idx]
            self.right_steering_angle = msg.position[right_idx]
        except ValueError:
            pass
    
    def verify_ackermann(self):
        """验证Ackermann几何"""
        rospy.sleep(1.0)
        
        test_angles = [0.0, 0.3, 0.6, -0.3, -0.6]
        
        for center_angle in test_angles:
            print(f"\n{'='*80}")
            print(f"测试: 发送 angular.z = {center_angle:+.2f} rad ({self.rad_to_deg(center_angle):+.1f}°)")
            print(f"{'='*80}")
            
            # 发送命令
            self.send_cmd(0.1, center_angle)
            rospy.sleep(1.5)
            
            if self.left_steering_angle is not None:
                # 计算理论值
                left_theory, right_theory = self.calculate_ackermann_theory(center_angle)
                
                # 实际值
                left_actual = self.left_steering_angle
                right_actual = self.right_steering_angle
                
                # 计算误差
                left_error = abs(left_actual - left_theory)
                right_error = abs(right_actual - right_theory)
                
                # 打印结果
                print(f"\n【转向角度】")
                print(f"左轮转向:")
                print(f"  理论值: {left_theory:+.4f} rad ({self.rad_to_deg(left_theory):+.2f}°)")
                print(f"  实际值: {left_actual:+.4f} rad ({self.rad_to_deg(left_actual):+.2f}°)")
                print(f"  误差:   {left_error:.4f} rad ({self.rad_to_deg(left_error):.2f}°)")
                
                print(f"\n右轮转向:")
                print(f"  理论值: {right_theory:+.4f} rad ({self.rad_to_deg(right_theory):+.2f}°)")
                print(f"  实际值: {right_actual:+.4f} rad ({self.rad_to_deg(right_actual):+.2f}°)")
                print(f"  误差:   {right_error:.4f} rad ({self.rad_to_deg(right_error):.2f}°)")
                
                # 检查轮速差异
                print(f"\n【差速检查】")
                if abs(center_angle) > 0.01:
                    print(f"转弯时左右轮速度应该不同(内侧慢,外侧快)")
                    print(f"  提示: 在Gazebo中观察轮子转速或检查joint_states话题")
                
                # 验证Ackermann关系
                if abs(center_angle) > 0.01:
                    print(f"\nAckermann验证:")
                    if center_angle > 0:
                        print(f"  左转: 左轮(内侧)应该比右轮(外侧)转角更大")
                        if abs(left_actual) > abs(right_actual):
                            print(f"  ✓ 正确: |{self.rad_to_deg(left_actual):.2f}°| > |{self.rad_to_deg(right_actual):.2f}°|")
                        else:
                            print(f"  ✗ 错误: |{self.rad_to_deg(left_actual):.2f}°| <= |{self.rad_to_deg(right_actual):.2f}°|")
                    else:
                        print(f"  右转: 右轮(内侧)应该比左轮(外侧)转角更大")
                        if abs(right_actual) > abs(left_actual):
                            print(f"  ✓ 正确: |{self.rad_to_deg(right_actual):.2f}°| > |{self.rad_to_deg(left_actual):.2f}°|")
                        else:
                            print(f"  ✗ 错误: |{self.rad_to_deg(right_actual):.2f}°| <= |{self.rad_to_deg(left_actual):.2f}°|")
                    
                    # 计算转弯半径
                    radius = self.calculate_turn_radius(left_actual, right_actual)
                    if radius is not None:
                        print(f"  转弯半径: {radius:.3f} m")
                else:
                    print(f"\n直行: 左右轮转角应该相同且接近0")
                    if left_error < 0.01 and right_error < 0.01:
                        print(f"  ✓ 正确")
                    else:
                        print(f"  ✗ 误差过大")
        
        print(f"\n{'='*80}")
        print("验证完成!")
        print(f"{'='*80}\n")
    
    def calculate_ackermann_theory(self, center_angle):
        """计算理论Ackermann角度"""
        if abs(center_angle) < 1e-6:
            return 0.0, 0.0
        
        offset = self.track_width / (2.0 * self.wheelbase)
        
        if center_angle > 0:  # Left turn
            cot_left = 1.0 / math.tan(center_angle) - offset
            left_angle = math.atan(1.0 / cot_left)
            
            cot_right = 1.0 / math.tan(center_angle) + offset
            right_angle = math.atan(1.0 / cot_right)
        else:  # Right turn
            cot_left = 1.0 / math.tan(center_angle) - offset
            left_angle = math.atan(1.0 / cot_left)
            
            cot_right = 1.0 / math.tan(center_angle) + offset
            right_angle = math.atan(1.0 / cot_right)
        
        return left_angle, right_angle
    
    def calculate_turn_radius(self, left_angle, right_angle):
        """计算转弯半径"""
        try:
            # 使用平均转角估算
            avg_angle = (left_angle + right_angle) / 2.0
            if abs(avg_angle) < 1e-6:
                return None
            radius = self.wheelbase / math.tan(abs(avg_angle))
            return radius
        except:
            return None
    
    def send_cmd(self, linear_x, angular_z):
        """发送速度命令"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
    
    @staticmethod
    def rad_to_deg(rad):
        """弧度转角度"""
        return rad * 180.0 / math.pi

if __name__ == '__main__':
    try:
        verifier = AckermannVerifier()
        verifier.verify_ackermann()
    except rospy.ROSInterruptException:
        pass
