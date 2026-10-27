#!/bin/bash
# 快速测试向量场规划器

cd /home/gcs/kitti_test
source devel/setup.bash

echo "=== 启动向量场规划器 ==="
rosrun bringup vector_field_planner.py &
PLANNER_PID=$!

sleep 3

echo ""
echo "=== 检查节点状态 ==="
rosnode list | grep vector

echo ""
echo "=== 检查话题 ==="
rostopic list | grep -E "(cmd_vel|vector_field)"

echo ""
echo "=== 检查/cmd_vel话题内容 ==="
timeout 2 rostopic echo /cmd_vel -n 1 || echo "没有消息"

echo ""
echo "=== 检查里程计数据 ==="
timeout 2 rostopic echo /lio_sam/mapping/odometry -n 1 | grep -A 10 "position:" || echo "没有里程计数据"

# 清理
kill $PLANNER_PID 2>/dev/null
