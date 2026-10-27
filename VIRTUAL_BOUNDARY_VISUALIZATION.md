# 虚拟障碍物边界可视化指南

## 发布的ROS话题

### 1. 障碍物边界可视化
**话题**: `/vector_field_planner/obstacle_boundaries`  
**类型**: `visualization_msgs/MarkerArray`  
**内容**:
- **黄色半透明圆柱体**: 原始障碍物（真实大小）
- **红色半透明圆柱体**: 排斥边界（repulsive_offset）
- **橙色半透明圆柱体**: 真实反应边界（reactive_offset）
- **蓝色半透明圆柱体**: 虚拟反应边界（virtual_reactive_offset）⭐ **新增**
- **白色文字**: 障碍物ID标签

### 2. 向量场可视化
**话题**: `/vector_field_planner/vector_field`  
**类型**: `visualization_msgs/MarkerArray`  
**内容**: 橙色箭头 - 复合向量场（最终导引向量）

**话题**: `/vector_field_planner/path_field`  
**类型**: `visualization_msgs/MarkerArray`  
**内容**: 绿色箭头 - 路径跟踪向量场

**话题**: `/vector_field_planner/reactive_field`  
**类型**: `visualization_msgs/MarkerArray`  
**内容**: 红色箭头 - 真实反应向量场（传统避障）

**话题**: `/vector_field_planner/virtual_field` ⭐ **新增**  
**类型**: `visualization_msgs/MarkerArray`  
**内容**: 蓝色箭头 - 虚拟排斥向量场（仅在缓冲区内显示）

### 3. 其他可视化
**话题**: `/vector_field_planner/current_vector`  
**类型**: `visualization_msgs/Marker`  
**内容**: 紫色大箭头 - 当前车辆位置的导引向量

**话题**: `/vector_field_planner/circle_path`  
**类型**: `visualization_msgs/Marker`  
**内容**: 红色线条 - 目标路径（圆形/线段/8字形）

**话题**: `/vector_field_planner/local_plan`  
**类型**: `nav_msgs/Path`  
**内容**: 局部规划路径（沿向量场积分生成）

---

## 在RViz中添加可视化

### 方法1: 手动添加（推荐用于测试）

1. 打开RViz
2. 点击左下角 `Add` 按钮
3. 选择 `By topic` 标签
4. 找到 `/vector_field_planner/` 相关话题

#### 添加虚拟边界（障碍物层）
- 选择 `/vector_field_planner/obstacle_boundaries`
- 类型: `MarkerArray`
- 点击 `OK`

#### 添加虚拟向量场（蓝色箭头）
- 选择 `/vector_field_planner/virtual_field`
- 类型: `MarkerArray`
- 点击 `OK`

### 方法2: 修改RViz配置文件

编辑 `src/racecar_sim-noetic/bringup/rviz/lio_sam_navigation.rviz`，添加以下配置：

```yaml
  - Class: rviz/MarkerArray
    Enabled: true
    Marker Topic: /vector_field_planner/virtual_field
    Name: Virtual Field
    Namespaces:
      virtual_arrows: true
    Queue Size: 100
```

---

## 颜色编码说明

| 颜色 | 含义 | 话题/命名空间 |
|------|------|--------------|
| 🟡 **黄色** | 原始障碍物 | `obstacle_original` |
| 🔴 **红色** | 排斥边界 | `repulsive_boundary` |
| 🟠 **橙色** | 真实反应边界 | `reactive_boundary` |
| 🔵 **蓝色** | 虚拟反应边界 ⭐ | `virtual_reactive_boundary` |
| 🟢 **绿色箭头** | 路径跟踪向量 | `/path_field` |
| 🔴 **红色箭头** | 真实反应向量 | `/reactive_field` |
| 🔵 **蓝色箭头** | 虚拟排斥向量 ⭐ | `/virtual_field` |
| 🟠 **橙色箭头** | 复合导引向量 | `/vector_field` |
| 🟣 **紫色大箭头** | 当前车辆向量 | `/current_vector` |

---

## 验证虚拟边界是否生效

### 1. 检查参数
```bash
# 查看虚拟边界开关状态
rosparam get /vector_field_planner_cpp/enable_virtual_boundary

# 查看虚拟反应半径
rosparam get /vector_field_planner_cpp/virtual_reactive_offset

# 查看衰减系数
rosparam get /vector_field_planner_cpp/k_c
```

### 2. 查看发布的话题
```bash
# 列出所有向量场话题
rostopic list | grep vector_field_planner

# 应该看到：
# /vector_field_planner/obstacle_boundaries
# /vector_field_planner/vector_field
# /vector_field_planner/path_field
# /vector_field_planner/reactive_field
# /vector_field_planner/virtual_field  ← 新增
# /vector_field_planner/current_vector
# ...
```

### 3. 监控话题发布频率
```bash
# 检查虚拟向量场是否在发布（仅在虚拟边界开启且机器人在缓冲区内时）
rostopic hz /vector_field_planner/virtual_field

# 检查障碍物边界（应该包含4层：原始+排斥+反应+虚拟）
rostopic echo /vector_field_planner/obstacle_boundaries | grep "ns:"
```

### 4. 可视化验证

在RViz中观察：
- **蓝色半透明圆柱体**应该是最外层，比橙色（真实反应边界）更大
- **蓝色箭头**应该只在缓冲区内显示（reactive_offset < d < virtual_reactive_offset）
- 当机器人接近障碍物时，应该在进入橙色边界之前就看到蓝色箭头的影响

---

## 测试场景

### 场景1: 禁用虚拟边界（基线测试）
```yaml
enable_virtual_boundary: false
```
**预期结果**:
- ❌ 没有蓝色圆柱体
- ❌ 没有蓝色箭头
- ✅ 只有黄/红/橙三层障碍物边界

### 场景2: 启用虚拟边界
```yaml
enable_virtual_boundary: true
virtual_reactive_offset: 1.2
k_c: 0.5
```
**预期结果**:
- ✅ 有蓝色圆柱体（最外层）
- ✅ 在缓冲区内看到蓝色箭头
- ✅ 车辆转向更平滑

---

## 常见问题排查

### Q: 看不到虚拟边界（蓝色圆柱体）？

**检查清单**:
1. ✅ 确认 `enable_virtual_boundary: true`
2. ✅ 重启节点: `rosnode kill /vector_field_planner_cpp && roslaunch ...`
3. ✅ 在RViz中添加 `/vector_field_planner/obstacle_boundaries` 话题
4. ✅ 确认场景中有障碍物（检查 `/obs_fitting/ellipse_equations`）

### Q: 看不到虚拟向量（蓝色箭头）？

**原因**: 虚拟向量只在缓冲区内显示！

**检查**:
1. ✅ 机器人是否在缓冲区内？（reactive_offset < d < virtual_reactive_offset）
2. ✅ 在RViz中添加 `/vector_field_planner/virtual_field` 话题
3. ✅ 尝试将机器人开到障碍物附近（但不要太近）

### Q: 向量场网格太密集，RViz卡顿？

**解决方案**:
```yaml
# 在 vector_field_planner_params.yaml 中调整
field_resolution: 0.5  # 增大间距（默认0.3）
field_range: 10.0      # 减小范围（默认15.0）
viz_frequency: 5.0     # 降低更新频率（默认10.0）
```

或者在主循环中注释掉向量场可视化：
```cpp
// visualizeVectorField();  // 已在 run() 函数中注释
```

---

## 性能优化建议

如果RViz性能不足，按优先级禁用：

1. **向量场网格** (最消耗资源):
   - `/vector_field_planner/vector_field`
   - `/vector_field_planner/path_field`
   - `/vector_field_planner/reactive_field`
   - `/vector_field_planner/virtual_field`

2. **保留关键可视化** (必需):
   - `/vector_field_planner/obstacle_boundaries` (障碍物边界)
   - `/vector_field_planner/current_vector` (当前导引向量)
   - `/vector_field_planner/local_plan` (局部路径)

---

## 命令速查

```bash
# 启动系统
roslaunch bringup circular_path_navigation_cpp.launch

# 动态修改参数（需重启节点）
rosparam set /vector_field_planner_cpp/enable_virtual_boundary true
rosparam set /vector_field_planner_cpp/virtual_reactive_offset 1.5
rosparam set /vector_field_planner_cpp/k_c 0.4
rosnode kill /vector_field_planner_cpp

# 查看所有可视化话题
rostopic list | grep vector_field_planner

# 实时监控虚拟边界效果
rostopic echo /vector_field_planner/obstacle_boundaries | grep "ns:"
rostopic hz /vector_field_planner/virtual_field
```

---

**最后更新**: 2025-12-27  
**版本**: 1.1 (添加虚拟边界可视化)
