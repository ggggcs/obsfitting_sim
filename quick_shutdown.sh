#!/bin/bash
# 快速关闭 ROS 和 Gazebo 的脚本

echo "🛑 快速关闭 ROS 节点..."

# 设置较短的关闭超时时间
export GAZEBO_MASTER_URI=http://localhost:11345
export ROS_SHUTDOWN_TIMEOUT=3

# 关闭所有 ROS 节点
rosnode kill -a 2>/dev/null

# 等待一小段时间
sleep 2

# 强制杀死 Gazebo 进程（如果还在运行）
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f rviz 2>/dev/null

echo "✅ 关闭完成"
