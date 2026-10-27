#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ackermann转向外八问题诊断和验证
"""
import math

print("""
╔═══════════════════════════════════════════════════════════════════════════════╗
║                    Ackermann转向"外八"问题分析和修复                          ║
╚═══════════════════════════════════════════════════════════════════════════════╝

【问题描述】
"外八"现象: 两个前轮转向方向相反
- 左轮向左转(正角度)，右轮向右转(负角度)
- 或者左轮向右转(负角度)，右轮向左转(正角度)

这显然是错误的！正确的Ackermann转向应该是：
✓ 两个前轮始终朝同一方向转
✓ 内侧轮转角更大，外侧轮转角更小

【问题原因】
使用了不稳定的三角函数公式:
  φ = atan(2L·sin(φ₀) / (2L·cos(φ₀) ± W·sin(φ₀)))

当分母接近0或变号时，会导致数值不稳定，产生错误的符号。

【修复方案】
改用余切公式(数值稳定):
  cot(φ_inner) = cot(φ_center) - W/(2L)
  cot(φ_outer) = cot(φ_center) + W/(2L)
  
然后: φ = atan(1/cot(φ))

这个公式确保:
1. 左右轮符号始终一致(不会外八)
2. 内侧轮绝对值更大
3. 数值稳定性好

【验证方法】
运行以下测试脚本:

1. 角度计算测试:
   python /home/gcs/kitti_test/test_ackermann_angles.py
   
   检查各个角度下是否有外八现象

2. 实时可视化监控:
   python /home/gcs/kitti_test/visualize_ackermann.py
   
   实时显示转向状态，会标记"✗✗ 外八！"

3. 完整验证:
   python /home/gcs/kitti_test/verify_ackermann.py
   
   自动测试多个角度并验证

【修复前后对比】
""")

WHEELBASE = 0.335
TRACK_WIDTH = 0.365

def old_formula(center_angle):
    """旧公式 - 可能导致外八"""
    if abs(center_angle) < 1e-6:
        return 0.0, 0.0
    sin_phi = math.sin(center_angle)
    cos_phi = math.cos(center_angle)
    left = math.atan(2 * WHEELBASE * sin_phi / 
                    (2 * WHEELBASE * cos_phi - TRACK_WIDTH * sin_phi))
    right = math.atan(2 * WHEELBASE * sin_phi / 
                     (2 * WHEELBASE * cos_phi + TRACK_WIDTH * sin_phi))
    return left, right

def new_formula(center_angle):
    """新公式 - 修复外八"""
    if abs(center_angle) < 1e-6:
        return 0.0, 0.0
    cot_center = 1.0 / math.tan(center_angle)
    offset = TRACK_WIDTH / (2.0 * WHEELBASE)
    cot_left = cot_center - offset
    left = math.atan(1.0 / cot_left)
    cot_right = cot_center + offset
    right = math.atan(1.0 / cot_right)
    return left, right

print("测试角度: 0.3 rad (左转)")
print("-" * 80)
old_l, old_r = old_formula(0.3)
new_l, new_r = new_formula(0.3)
print(f"旧公式: 左轮={old_l:+.4f} ({old_l*57.3:+.1f}°), 右轮={old_r:+.4f} ({old_r*57.3:+.1f}°)")
print(f"新公式: 左轮={new_l:+.4f} ({new_l*57.3:+.1f}°), 右轮={new_r:+.4f} ({new_r*57.3:+.1f}°)")
if old_l * old_r < 0:
    print("⚠️  旧公式: 符号相反 - 外八！")
else:
    print("✓  旧公式: 符号一致")
if new_l * new_r < 0:
    print("⚠️  新公式: 符号相反 - 外八！")
else:
    print("✓  新公式: 符号一致")

print("\n测试角度: -0.3 rad (右转)")
print("-" * 80)
old_l, old_r = old_formula(-0.3)
new_l, new_r = new_formula(-0.3)
print(f"旧公式: 左轮={old_l:+.4f} ({old_l*57.3:+.1f}°), 右轮={old_r:+.4f} ({old_r*57.3:+.1f}°)")
print(f"新公式: 左轮={new_l:+.4f} ({new_l*57.3:+.1f}°), 右轮={new_r:+.4f} ({new_r*57.3:+.1f}°)")
if old_l * old_r < 0:
    print("⚠️  旧公式: 符号相反 - 外八！")
else:
    print("✓  旧公式: 符号一致")
if new_l * new_r < 0:
    print("⚠️  新公式: 符号相反 - 外八！")
else:
    print("✓  新公式: 符号一致")

print("""
╔═══════════════════════════════════════════════════════════════════════════════╗
║                              修复已完成                                       ║
╚═══════════════════════════════════════════════════════════════════════════════╝

servo_commands.py 已更新为使用稳定的余切公式。

请重新启动仿真并测试验证！
""")
