#!/bin/bash

echo "======================================"
echo "TF坐标系检查工具"
echo "======================================"
echo ""

# 检查是否有roscore运行
if ! pgrep -x "roscore" > /dev/null && ! pgrep -x "rosmaster" > /dev/null; then
    echo "错误: ROS master未运行!"
    echo "请先启动系统: ./start_lio_sam_navigation.sh"
    exit 1
fi

echo "1. 检查当前TF树..."
echo ""

# 列出所有TF frames
echo "当前活动的坐标系："
timeout 5 rostopic echo /tf --noarr 2>/dev/null | grep "frame_id\|child_frame_id" | sort -u | head -20

echo ""
echo "======================================"
echo "2. 检查关键TF转换..."
echo ""

# 检查map到odom
echo "检查 map -> odom 转换："
timeout 2 rosrun tf tf_echo map odom 2>&1 | head -5

echo ""

# 检查odom到base_link
echo "检查 odom -> base_link 转换："
timeout 2 rosrun tf tf_echo odom base_link 2>&1 | head -5

echo ""

# 检查base_link到base_footprint
echo "检查 base_link -> base_footprint 转换："
timeout 2 rosrun tf tf_echo base_link base_footprint 2>&1 | head -5

echo ""
echo "======================================"
echo "3. 生成完整TF树图..."
echo ""

cd /tmp
rosrun tf view_frames 2>&1 | grep -E "Listening|dot"

if [ -f /tmp/frames.pdf ]; then
    echo "TF树图已生成: /tmp/frames.pdf"
    echo "使用以下命令查看:"
    echo "  evince /tmp/frames.pdf"
fi

echo ""
echo "======================================"
echo "检查完成！"
echo "======================================"
