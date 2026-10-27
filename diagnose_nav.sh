#!/bin/bash
# 导航系统诊断脚本

echo "=========================================="
echo "导航系统诊断"
echo "=========================================="

source /home/gcs/kitti_test/devel/setup.bash

echo ""
echo "1. 检查关键节点:"
rosnode list | grep -E "move_base|pointcloud_to_laserscan|nav_sim" || echo "  ❌ 某些节点未运行"

echo ""
echo "2. 检查2D扫描话题:"
timeout 2 rostopic hz /scan 2>&1 | grep "average rate" || echo "  ❌ /scan话题无数据"

echo ""
echo "3. 检查路径规划话题:"
echo "  Global Plan:"
timeout 1 rostopic echo /move_base/DWAPlannerROS/global_plan -n 1 2>&1 | head -3

echo ""
echo "4. 检查move_base状态:"
timeout 1 rostopic echo /move_base/status -n 1 2>&1 | grep "status:" | head -1

echo ""
echo "5. 检查TF坐标系:"
rosrun tf tf_echo base_link laser 2>&1 | grep -E "Translation|Rotation" | head -2 || echo "  ⚠️  base_link到laser的TF可能有问题"

echo ""
echo "=========================================="
echo "诊断完成"
echo "=========================================="
