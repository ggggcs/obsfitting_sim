# 向量场规划器路径类型使用指南

## 概述

向量场规划器现在支持三种路径类型：
1. **圆形路径 (circle)** - 沿圆形轨迹运动
2. **线段路径 (line)** - 沿有限线段轨迹运动（有起点和终点）
3. **8字形路径 (figure8)** - 沿8字形轨迹运动

## 配置方法

编辑配置文件：`src/racecar_sim-noetic/bringup/param/vector_field_planner_params.yaml`

### 1. 圆形路径 (Circle Path)

```yaml
path_type: 'circle'
circle_center_x: 0.0    # 圆心X坐标 (米)
circle_center_y: 0.0    # 圆心Y坐标 (米)
circle_radius: 4.0      # 圆半径 (米)
circle_direction: 'ccw' # 运动方向: 'ccw' (逆时针) 或 'cw' (顺时针)
```

**隐式函数**: `(x-cx)² + (y-cy)² = R²`

**适用场景**:
- 环形巡逻
- 测试避障性能
- 封闭路径跟踪

**示例配置**:
- 半径4米的圆: `circle_radius: 4.0`
- 圆心偏移: `circle_center_x: 2.0, circle_center_y: 3.0`
- 顺时针: `circle_direction: 'cw'`

### 2. 线段路径 (Line Segment Path)

```yaml
path_type: 'line'
line_start_x: -5.0       # 线段起点X坐标 (米)
line_start_y: 0.0        # 线段起点Y坐标 (米)
line_end_x: 5.0          # 线段终点X坐标 (米)
line_end_y: 0.0          # 线段终点Y坐标 (米)
line_direction: 'forward' # 运动方向: 'forward' 或 'backward'
```

**隐式函数**: 到线段的带符号距离函数

**适用场景**:
- 走廊巡航（有限长度）
- 点到点导航
- 往返巡逻

**常用示例**:

| 路径类型 | 起点 (x,y) | 终点 (x,y) | 说明 |
|---------|-----------|-----------|------|
| 水平线段 | (-5, 0) | (5, 0) | 沿X轴，长10米 |
| 垂直线段 | (0, -3) | (0, 3) | 沿Y轴，长6米 |
| 对角线段 | (-4, -4) | (4, 4) | 45度斜线 |
| 任意线段 | (1, 2) | (5, 8) | 自定义起止点 |

**方向说明**:
- `forward`: 从起点向终点运动
- `backward`: 从终点向起点运动

**特点**:
- 车辆会自动收敛到线段上最近的点
- 不会超出线段端点范围
- 可视化显示起点（绿色球）和终点（红色球）

### 3. 8字形路径 (Figure-8 Path)

```yaml
path_type: 'figure8'
figure8_scale: 3.0      # 8字形大小缩放系数 (米)
figure8_direction: 'ccw' # 运动方向: 'ccw' (逆时针) 或 'cw' (顺时针)
```

**隐式函数**: `(x² + y²)² = a²(x² - y²)`

**参数方程**: `x = a·cos(t), y = a·sin(2t)/2`

**适用场景**:
- 复杂轨迹跟踪
- 算法测试
- 展示用途

**尺寸说明**:
- `figure8_scale: 3.0` → 8字形最大宽度约6米
- `figure8_scale: 5.0` → 8字形最大宽度约10米

## 快速切换路径

只需修改 `path_type` 参数，然后重启节点：

```bash
# 1. 编辑配置文件
nano src/racecar_sim-noetic/bringup/param/vector_field_planner_params.yaml

# 2. 重启规划器
roslaunch bringup circular_path_navigation_cpp.launch
```

## 性能调优建议

### 通用参数
- `k_p`: 路径跟踪增益 (建议 0.05-0.2)
- `max_speed`: 最大速度 (根据路径复杂度调整)
- `steering_gain`: 转向增益 (直线路径可适当提高)

### 路径特定调优

**圆形路径**:
- 小半径 (<2m): 降低速度，增加 `k_p`
- 大半径 (>5m): 可提高速度

**线段路径**:
- 提高 `steering_gain` 以加快收敛到线段
- 适当降低 `k_p` 避免震荡
- 注意：接近端点时可能需要手动处理（未来可添加端点行为）

**8字形路径**:
- 保持中等速度
- 可能需要调整 `k_p` 以平衡内外环

## 可视化

所有路径类型都会在RViz中以红色线条显示：
- Topic: `/vector_field_planner/circle_path`
- Frame: `map`
- 颜色: 红色

查看向量场：
- 复合向量场: `/vector_field_planner/vector_field` (橙色)
- 路径跟踪: `/vector_field_planner/path_field` (绿色)
- 避障反应: `/vector_field_planner/reactive_field` (红色)

## 故障排查

### 问题：车辆不运动
- 检查路径参数是否合理（如圆半径不能为0）
- 确认 `max_speed > 0`
- 查看日志中的路径类型确认

### 问题：路径跟踪不稳定
- 调整 `k_p` 增益
- 检查 `steering_gain` 是否过大
- 验证路径方程参数正确

### 问题：线段路径到端点后停止
- 这是设计行为，线段有明确的起点和终点
- 如需往返，可在到达端点后手动切换 `line_direction`
- 或者使用圆形路径替代

### 问题：8字形路径不闭合
- 这是正常的，隐式方程和参数方程可能略有差异
- 调整 `figure8_scale` 改变形状

## 示例配置文件

### 示例1: 大圆巡逻
```yaml
path_type: 'circle'
circle_center_x: 0.0
circle_center_y: 0.0
circle_radius: 8.0
circle_direction: 'ccw'
max_speed: 1.0
k_p: 0.1
```

### 示例2: 水平线段往返
```yaml
path_type: 'line'
line_start_x: -6.0
line_start_y: 0.0
line_end_x: 6.0
line_end_y: 0.0
line_direction: 'forward'
max_speed: 0.8
k_p: 0.08
steering_gain: 1.8
```

### 示例3: 8字形展示
```yaml
path_type: 'figure8'
figure8_scale: 4.0
figure8_direction: 'ccw'
max_speed: 0.6
k_p: 0.15
```

## 数学背景

### 向量场生成公式
对于任意隐式路径 `φ(x,y) = 0`，向量场定义为：

```
χ_P = K·E·∇φ - k_p·φ·∇φ
```

其中：
- `E` 是旋转矩阵 `[[0, -1], [1, 0]]`
- `K` 是方向系数 (+1 或 -1)
- `∇φ` 是路径函数的梯度

这确保了：
1. 在路径上 (`φ=0`)，车辆沿切线方向运动
2. 偏离路径时，产生向路径收敛的分量

## 扩展开发

要添加新的路径类型：

1. 在 `vector_field_planner_params.yaml` 添加参数
2. 在 `vector_field_planner.h` 添加成员变量和函数声明
3. 在 `vector_field_planner.cpp` 实现路径函数
4. 在 `vector_field_planner_viz.cpp` 添加可视化函数
5. 更新 `pathTrackingVectorField()` 添加新类型判断

路径函数模板：
```cpp
void MyPathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi) {
    // 计算隐式函数值
    phi_val = /* your implicit function */;
    
    // 计算梯度
    grad_phi << /* ∂φ/∂x */, /* ∂φ/∂y */;
}
```
