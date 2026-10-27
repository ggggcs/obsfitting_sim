#!/bin/bash
# 测试圆形路径可视化

echo "=== 检查向量场规划器参数 ==="
echo "圆心: $(rosparam get /vector_field_planner/circle_center_x 2>/dev/null || echo '未设置'), $(rosparam get /vector_field_planner/circle_center_y 2>/dev/null || echo '未设置')"
echo "半径: $(rosparam get /vector_field_planner/circle_radius 2>/dev/null || echo '未设置')"
echo "方向: $(rosparam get /vector_field_planner/circle_direction 2>/dev/null || echo '未设置')"

echo ""
echo "=== 检查相关话题 ==="
rostopic list | grep -E "(circle_path|vector_field|cmd_vel)" || echo "没有找到相关话题"

echo ""
echo "=== 检查圆形路径话题 ==="
timeout 2 rostopic info /vector_field_planner/circle_path 2>/dev/null || echo "圆形路径话题不存在"

echo ""
echo "=== 在RViz中添加以下Display ==="
echo "1. Marker -> Topic: /vector_field_planner/circle_path (红色圆圈)"
echo "2. MarkerArray -> Topic: /vector_field_planner/vector_field (向量场)"
echo "3. Marker -> Topic: /vector_field_planner/current_vector (紫色箭头)"
echo "4. MarkerArray -> Topic: /vector_field_planner/obstacle_boundaries (障碍物)"
echo "5. Path -> Topic: /vector_field_planner/local_plan (局部路径)"

echo ""
echo "=== 提示 ==="
echo "如果看不到圆形路径，请确保："
echo "1. 向量场规划器节点正在运行: rosnode list | grep vector_field_planner"
echo "2. 圆形路径话题有订阅者（RViz）"
echo "3. RViz的Fixed Frame设置为 'map'"
