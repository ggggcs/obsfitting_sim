# 线段路径配置示例

## 快速使用

### 示例1: 水平线段（10米）
```yaml
path_type: 'line'
line_start_x: -5.0
line_start_y: 0.0
line_end_x: 5.0
line_end_y: 0.0
line_direction: 'forward'
```
**效果**: 从(-5,0)沿X轴向右移动到(5,0)

### 示例2: 垂直线段（6米）
```yaml
path_type: 'line'
line_start_x: 0.0
line_start_y: -3.0
line_end_x: 0.0
line_end_y: 3.0
line_direction: 'forward'
```
**效果**: 从(0,-3)沿Y轴向上移动到(0,3)

### 示例3: 对角线段
```yaml
path_type: 'line'
line_start_x: -4.0
line_start_y: -4.0
line_end_x: 4.0
line_end_y: 4.0
line_direction: 'forward'
```
**效果**: 从左下(-4,-4)向右上(4,4)移动，45度角

### 示例4: 任意角度线段
```yaml
path_type: 'line'
line_start_x: 0.0
line_start_y: 0.0
line_end_x: 5.0
line_end_y: 3.0
line_direction: 'forward'
```
**效果**: 从原点(0,0)移动到(5,3)

### 示例5: 反向运动
```yaml
path_type: 'line'
line_start_x: -5.0
line_start_y: 0.0
line_end_x: 5.0
line_end_y: 0.0
line_direction: 'backward'
```
**效果**: 从(5,0)向左移动到(-5,0)（反向）

## 可视化说明

在RViz中，线段路径会显示为：
- **红色线段**: 路径本身
- **绿色球**: 起点位置
- **红色球**: 终点位置

## 参数调优建议

### 短线段（<5米）
```yaml
max_speed: 0.5
k_p: 0.1
steering_gain: 2.0
```

### 中等线段（5-10米）
```yaml
max_speed: 0.8
k_p: 0.08
steering_gain: 1.5
```

### 长线段（>10米）
```yaml
max_speed: 1.0
k_p: 0.05
steering_gain: 1.2
```

## 实际应用场景

### 1. 走廊巡航
```yaml
# 20米走廊
path_type: 'line'
line_start_x: 0.0
line_start_y: 0.0
line_end_x: 20.0
line_end_y: 0.0
line_direction: 'forward'
max_speed: 0.6
```

### 2. 仓库货架间导航
```yaml
# 8米货架通道
path_type: 'line'
line_start_x: 2.0
line_start_y: 0.0
line_end_x: 2.0
line_end_y: 8.0
line_direction: 'forward'
max_speed: 0.5
k_p: 0.12  # 提高精度
```

### 3. 室内对角线穿越
```yaml
# 从房间一角到另一角
path_type: 'line'
line_start_x: -3.0
line_start_y: -2.0
line_end_x: 3.0
line_end_y: 2.0
line_direction: 'forward'
max_speed: 0.7
```

## 测试步骤

1. 修改配置文件：
```bash
nano src/racecar_sim-noetic/bringup/param/vector_field_planner_params.yaml
```

2. 重启节点：
```bash
roslaunch bringup circular_path_navigation_cpp.launch
```

3. 在RViz中观察：
   - 线段路径（红色线段 + 端点球）
   - 车辆当前位置
   - 向量场箭头

4. 确认行为：
   - 车辆应自动收敛到线段
   - 沿线段方向运动
   - 接近终点时减速或停止

## 注意事项

⚠️ **重要提示**:
- 线段路径是**有限的**，车辆到达终点后不会自动返回
- 如需往返运动，考虑使用圆形路径或手动切换方向
- 车辆初始位置最好在线段附近（<5米）以便快速收敛
- 线段长度建议 ≥ 3米，太短可能导致控制不稳定

## 与圆形路径对比

| 特性 | 线段路径 | 圆形路径 |
|------|---------|---------|
| 终止条件 | 有终点 | 无限循环 |
| 适用场景 | 点到点导航 | 连续巡逻 |
| 参数数量 | 4个坐标 | 3个参数 |
| 可视化 | 线段+端点 | 完整圆环 |
| 稳定性 | 接近端点时需注意 | 全程稳定 |

## 数学原理

线段路径使用**到线段的最短距离**作为隐式函数：

```
给定线段: P1(x1,y1) → P2(x2,y2)
对于任意点 Q(x,y):
1. 计算投影参数 t = (Q-P1)·(P2-P1) / ||P2-P1||²
2. 限制 t ∈ [0,1] 确保在线段范围内
3. 最近点 C = P1 + t*(P2-P1)
4. 距离 φ = ||Q-C|| (带符号)
5. 梯度 ∇φ = (Q-C)/||Q-C||
```

向量场在线段上产生切向分量，在线段外产生收敛分量。
