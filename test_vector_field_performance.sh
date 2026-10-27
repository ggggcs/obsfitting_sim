#!/bin/bash
# 性能对比测试脚本

echo "============================================"
echo "向量场规划器性能对比测试"
echo "============================================"
echo ""

# 检查是否已编译
if [ ! -f "/home/gcs/kitti_test/devel/lib/bringup/vector_field_planner_cpp" ]; then
    echo "错误: C++可执行文件不存在，请先编译："
    echo "  cd /home/gcs/kitti_test"
    echo "  catkin_make --only-pkg-with-deps bringup"
    exit 1
fi

echo "✓ C++可执行文件已就绪"
echo ""

# Source环境
source /home/gcs/kitti_test/devel/setup.bash

echo "使用说明："
echo "----------"
echo "1. 首先运行Python版本，观察性能话题："
echo "   roslaunch bringup circular_path_navigation.launch"
echo ""
echo "   在另一个终端监控性能："
echo "   rostopic echo /vector_field_planner/field_compute_time"
echo "   rostopic echo /vector_field_planner/control_query_time"
echo ""
echo "2. 停止Python版本（Ctrl+C），然后运行C++版本："
echo "   roslaunch bringup circular_path_navigation_cpp.launch"
echo ""
echo "   同样监控性能话题，对比耗时差异"
echo ""
echo "预期结果："
echo "----------"
echo "Python版本: field_compute_time ≈ 50-100ms"
echo "C++版本:    field_compute_time ≈ 3-10ms"
echo "性能提升:   约10-20倍"
echo ""
echo "============================================"
echo ""

# 提供快速启动选项
while true; do
    echo "请选择测试模式："
    echo "  1) 启动Python版本"
    echo "  2) 启动C++版本"
    echo "  3) 监控性能话题（新终端）"
    echo "  4) 退出"
    echo ""
    read -p "请输入选项 (1-4): " choice
    
    case $choice in
        1)
            echo ""
            echo "启动Python版本..."
            roslaunch bringup circular_path_navigation.launch
            ;;
        2)
            echo ""
            echo "启动C++版本..."
            roslaunch bringup circular_path_navigation_cpp.launch
            ;;
        3)
            echo ""
            echo "请在新终端中运行以下命令："
            echo ""
            echo "# 监控向量场计算时间"
            echo "rostopic echo /vector_field_planner/field_compute_time"
            echo ""
            echo "# 监控控制查询时间"
            echo "rostopic echo /vector_field_planner/control_query_time"
            echo ""
            echo "# 监控可视化时间"
            echo "rostopic echo /vector_field_planner/viz_time"
            echo ""
            echo "# 对比话题频率"
            echo "rostopic hz /cmd_vel"
            echo ""
            read -p "按Enter继续..."
            ;;
        4)
            echo "退出测试"
            exit 0
            ;;
        *)
            echo "无效选项，请重新选择"
            ;;
    esac
    
    echo ""
done
