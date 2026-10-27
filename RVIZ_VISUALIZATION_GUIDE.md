# RViz可视化配置指南

## 查看圆形路径和向量场

### 需要添加的Display

在RViz中，点击左下角的 "Add" 按钮，添加以下显示项：

#### 1. 圆形路径（红色圆圈）
- **Display Type**: `Marker`
- **Topic**: `/vector_field_planner/circle_path`
- **颜色**: 红色
- **说明**: 显示期望的圆形轨迹

#### 2. 向量场（灰色箭头网格）
- **Display Type**: `MarkerArray`
- **Topic**: `/vector_field_planner/vector_field`
- **说明**: 显示整个向量场，箭头指示每个位置的运动方向

#### 3. 当前向量（紫色大箭头）
- **Display Type**: `Marker`
- **Topic**: `/vector_field_planner/current_vector`
- **颜色**: 紫色
- **说明**: 显示车辆当前位置的控制向量

#### 4. 障碍物边界（椭圆）
- **Display Type**: `MarkerArray`
- **Topic**: `/vector_field_planner/obstacle_boundaries`
- **说明**: 显示障碍物的排斥边界（红色）和反应边界（橙色）

#### 5. 局部路径（蓝色线）
- **Display Type**: `Path`
- **Topic**: `/vector_field_planner/local_plan`
- **颜色**: 蓝色
- **说明**: 显示通过向量场积分生成的局部路径

#### 6. 车辆位置
- **Display Type**: `RobotModel`
- **说明**: 显示车辆模型

#### 7. 地图（如果使用LIO-SAM）
- **Display Type**: `PointCloud2`
- **Topic**: `/lio_sam/mapping/map_global`
- **说明**: 显示SLAM构建的点云地图

## 快速配置命令

```bash
# 启动系统
roslaunch bringup implicit_path_navigation.launch

# 在RViz中，你应该能看到以下话题：
rostopic list | grep vector_field_planner
```

预期输出：
```
/vector_field_planner/circle_path          # 圆形路径
/vector_field_planner/current_vector       # 当前控制向量
/vector_field_planner/local_plan           # 局部路径
/vector_field_planner/obstacle_boundaries  # 障碍物边界
/vector_field_planner/vector_field         # 完整向量场
/vector_field_planner/yaw_error            # 航向误差（调试用）
```

## 检查话题

```bash
# 1. 查看圆形路径
rostopic echo /vector_field_planner/circle_path -n 1

# 2. 查看控制命令
rostopic echo /cmd_vel

# 3. 查看向量场规划器日志
# 应该看到类似：
# [INFO] 圆形路径可视化已发布到 /vector_field_planner/circle_path
# [INFO] 位置: (x,y), chi_P原始norm=..., chi_c=...
```

## RViz配置文件

如果RViz配置丢失，可以手动设置：

1. **Fixed Frame**: 设置为 `map`
2. **添加上述所有Display**
3. **保存配置**: File -> Save Config As -> `lio_sam_navigation.rviz`

## 调试提示

### 如果看不到圆形路径
```bash
# 检查话题是否有数据
rostopic info /vector_field_planner/circle_path

# 检查节点是否运行
rosnode list | grep vector_field_planner
```

### 如果看不到向量场
- 确保车辆已经接收到里程计数据
- 检查 `/lio_sam/mapping/odometry` 是否有数据
- 向量场只在车辆周围3-5米范围内显示

### 颜色说明
- 🔴 **红色**: 期望路径（圆形）
- 🟣 **紫色**: 当前控制向量（大箭头）
- ⚫ **灰色**: 向量场网格（小箭头）
- 🟠 **橙色**: 障碍物反应边界
- 🔴 **红色**: 障碍物排斥边界
- 🟡 **黄色**: 障碍物本体
- 🔵 **蓝色**: 局部规划路径

## 常用视角设置

- **俯视图**: 设置Camera Type为 `TopDownOrtho`，适合查看路径
- **跟随车辆**: 设置Target Frame为 `base_link`，适合观察车辆行为
- **全局视角**: 设置Fixed Frame为 `map`，Target Frame为空

## 动态调整参数

运行中可以调整参数：
```bash
# 调整圆半径
rosparam set /vector_field_planner/circle_radius 8.0

# 调整路径跟踪增益
rosparam set /vector_field_planner/k_p 1.5

# 重启规划器以应用新参数
rosnode kill /vector_field_planner
rosrun bringup vector_field_planner.py
```
