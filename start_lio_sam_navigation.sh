#!/bin/bash

# LIO-SAM + Navigation 启动脚本

echo "======================================"
echo "启动 LIO-SAM 实时建图导航系统"
echo "======================================"
echo ""
echo "系统组成："
echo "  1. Gazebo 仿真环境"
echo "  2. LIO-SAM SLAM 建图与定位"
echo "  3. Move_Base 路径规划"
echo "  4. RViz 可视化"
echo ""
echo "使用说明："
echo "  - 在RViz中使用 '2D Nav Goal' 设置导航目标"
echo "  - 小车会自动规划路径并导航"
echo "  - LIO-SAM会实时构建点云地图"
echo ""
echo "======================================"

# 进入工作空间
cd "$(dirname "$0")"

# Source环境
source devel/setup.bash

# 启动系统
roslaunch bringup lio_sam_navigation.launch
