# 圆形路径导航系统使用说明

## 概述
这是一个简化的导航系统，使用**圆形路径**替代A*全局规划器，配合**向量场局部规划器**实现避障导航。

## 系统架构

```
┌─────────────────────┐
│  圆形路径发布器      │ → 发布圆形路径
│ circular_path_pub   │   到 /move_base/NavfnROS/plan
└─────────────────────┘
          ↓
┌─────────────────────┐
│  向量场局部规划器    │ ← 订阅圆形路径
│ vector_field_plan   │ ← 订阅障碍物信息
│                     │ ← 订阅里程计
└─────────────────────┘
          ↓
    发布 /cmd_vel
          ↓
┌─────────────────────┐
│    导航接口节点      │ → 转换为Ackermann命令
│      nav_sim        │
└─────────────────────┘
```

## 启动方法

### 1. 基本启动
```bash
roslaunch bringup circular_path_navigation.launch
```

### 2. 自定义圆形路径参数
编辑 `param/circular_path_params.yaml`:

```yaml
center_x: 0.0       # 圆心X坐标 (米)
center_y: 0.0       # 圆心Y坐标 (米)
radius: 5.0         # 圆半径 (米)
num_points: 100     # 路径点数量
direction: 'ccw'    # 'cw'顺时针 或 'ccw'逆时针
```

### 3. 调整向量场参数
编辑 `param/vector_field_planner_params.yaml`:

```yaml
k_p: 1.0            # 路径跟踪增益
k_r: 1.5            # 反应向量场增益
max_speed: 2.0      # 最大速度
repulsive_offset: 0.6   # 排斥边界偏移
reactive_offset: 1.0    # 反应边界偏移
```

## 与原版的区别

### 原版 (vector_field_navigation.launch)
- 使用 move_base 的 A* 算法进行全局路径规划
- 全局代价地图 + 局部代价地图
- 向量场作为局部规划器

### 简化版 (circular_path_navigation.launch)
- ❌ 移除 move_base
- ❌ 移除 A* 全局规划器
- ❌ 移除代价地图
- ✅ 使用固定的圆形路径
- ✅ 保留向量场局部规划器
- ✅ 更简单、更易调试

## 可视化

在RViz中可以看到：
1. **圆形路径**: 蓝色线条 (话题: `/move_base/NavfnROS/plan`)
2. **向量场**: 箭头网格 (话题: `/vector_field_planner/vector_field`)
3. **当前向量**: 紫色大箭头 (话题: `/vector_field_planner/current_vector`)
4. **障碍物边界**: 红色/橙色椭圆 (话题: `/vector_field_planner/obstacle_boundaries`)

## 测试步骤

1. **启动系统**:
   ```bash
   roslaunch bringup circular_path_navigation.launch
   ```

2. **检查话题**:
   ```bash
   # 查看圆形路径
   rostopic echo /move_base/NavfnROS/plan -n 1
   
   # 查看控制命令
   rostopic echo /cmd_vel
   ```

3. **调整参数**:
   - 修改圆心位置，让小车在不同位置绕圈
   - 修改半径大小，测试不同圆形路径
   - 修改方向，测试顺时针/逆时针

## 扩展建议

### 更换路径类型
如果想使用其他路径形状（如直线、8字形等），可以修改 `circular_path_publisher.py`:

```python
def generate_figure_eight_path(self):
    """生成8字形路径"""
    # 实现你的路径生成逻辑
    pass
```

### 添加路径切换
可以通过服务或参数动态切换路径类型：
```python
rospy.Service('switch_path', PathSwitch, self.switch_path_callback)
```

## 故障排除

1. **小车不动**:
   - 检查 `/cmd_vel` 是否有数据
   - 检查向量场规划器是否收到路径

2. **路径不显示**:
   - 确认 RViz 订阅了正确的话题
   - 检查 `circular_path_publisher` 节点是否运行

3. **避障不工作**:
   - 确认 `obs_fitting` 节点正常运行
   - 检查障碍物话题 `/obs_fitting/ellipse_equations`

## 文件清单

- `scripts/circular_path_publisher.py` - 圆形路径发布器
- `param/circular_path_params.yaml` - 路径参数配置
- `launch/circular_path_navigation.launch` - 启动文件
- `README_CIRCULAR_PATH.md` - 本文档
