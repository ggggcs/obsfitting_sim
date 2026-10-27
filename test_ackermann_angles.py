#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试Ackermann转向角度计算
检查是否会出现"外八"现象
"""
import math

WHEELBASE = 0.335
TRACK_WIDTH = 0.365

def test_ackermann_angles():
    """测试不同输入角度下的转向计算"""
    
    print("\n" + "="*110)
    print("Ackermann转向角度计算测试 - 检查外八问题")
    print("="*110)
    print(f"车辆参数: 轴距L={WHEELBASE}m, 轮距W={TRACK_WIDTH}m")
    print("="*110)
    print(f"{'输入角度':^15} | {'左轮角度':^18} | {'右轮角度':^18} | {'内外轮':^12} | {'符号':^8} | {'状态':^15}")
    print("-"*110)
    
    # 测试角度范围
    test_angles = [0.0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.6, 
                   -0.01, -0.05, -0.1, -0.2, -0.3, -0.5, -0.6]
    
    has_error = False
    
    for center_angle in test_angles:
        if abs(center_angle) < 1e-6:
            left_angle = 0.0
            right_angle = 0.0
        else:
            # 使用余切公式 (修复后的方法)
            cot_center = 1.0 / math.tan(center_angle)
            offset = TRACK_WIDTH / (2.0 * WHEELBASE)
            
            cot_left = cot_center - offset
            left_angle = math.atan(1.0 / cot_left)
            
            cot_right = cot_center + offset
            right_angle = math.atan(1.0 / cot_right)
        
        # 转换为角度
        center_deg = center_angle * 180.0 / math.pi
        left_deg = left_angle * 180.0 / math.pi
        right_deg = right_angle * 180.0 / math.pi
        
        # 检查符号一致性
        sign_match = (left_angle * right_angle >= 0)  # 同号或有一个为0
        
        # 确定内外轮
        if abs(center_angle) < 0.01:
            inner_outer = "直行"
            sign_status = "N/A"
            status = "✓ 正常"
        elif center_angle > 0:  # 左转
            if abs(left_angle) > abs(right_angle):
                inner_outer = "左内>右外"
                sign_status = "✓ 同号" if sign_match else "✗ 异号"
                status = "✓ 正常" if sign_match else "✗ 外八！"
            else:
                inner_outer = "错误分配"
                sign_status = "✗"
                status = "✗ 内外轮反了"
        else:  # 右转
            if abs(right_angle) > abs(left_angle):
                inner_outer = "右内>左外"
                sign_status = "✓ 同号" if sign_match else "✗ 异号"
                status = "✓ 正常" if sign_match else "✗ 外八！"
            else:
                inner_outer = "错误分配"
                sign_status = "✗"
                status = "✗ 内外轮反了"
        
        # 检查外八（符号相反）
        if not sign_match and abs(left_angle) > 0.01 and abs(right_angle) > 0.01:
            has_error = True
        
        print(f"{center_deg:+7.2f}° ({center_angle:+.3f}) | "
              f"{left_deg:+7.2f}° ({left_angle:+.4f}) | "
              f"{right_deg:+7.2f}° ({right_angle:+.4f}) | "
              f"{inner_outer:^12} | "
              f"{sign_status:^8} | "
              f"{status:^15}")
    
    print("="*110)
    
    if has_error:
        print("\n❌ 发现问题: 存在外八现象(左右轮转向方向相反)!")
    else:
        print("\n✅ 测试通过: 所有角度下左右轮转向方向一致")
    
    print("\n说明:")
    print("• 左转: 左轮(内侧)角度应大于右轮(外侧)，且都为正值")
    print("• 右转: 右轮(内侧)角度应大于左轮(外侧)，且都为负值")
    print("• 外八: 左右轮转向方向相反(一个正一个负) - 这是错误的!")
    print("• 同号✓: 左右轮转向方向一致")
    print("="*110 + "\n")

if __name__ == '__main__':
    test_ackermann_angles()
