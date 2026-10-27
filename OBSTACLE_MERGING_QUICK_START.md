# 障碍物融合功能快速使用指南

## 功能说明
实现基于反应边界 OBB 的智能障碍物融合，自动合并重叠障碍物，提高向量场规划器的稳定性。

## 核心原则
- **触发条件**: 仅基于反应边界 (reactive boundary) OBB 的重叠
- **完全替代**: 删除所有重叠的原始障碍物，用唯一新椭圆替代
- **参数逆推**: 从 MinAreaRect 反推物理椭圆参数

## 快速启动

### 1. 编译
```bash
cd /home/gcs/kitti_test
catkin_make -DVTK_DIR=/usr/local/lib/cmake/vtk-9.4 -DPCL_SHARED_LIBS=ON
source devel/setup.bash
```

### 2. 运行
```bash
roslaunch bringup circular_path_navigation_cpp.launch
```

### 3. 查看可视化
- 自动弹出窗口: **"Obstacle Merge Debugger"**
- 显示融合前后对比和计算过程

## 可视化图层

| 颜色 | 含义 | 说明 |
|------|------|------|
| 灰色虚线 | 被删除的原始障碍物 | 显示融合前状态 |
| 蓝色细线 | 凸包和最小外接矩形 | 几何计算过程 |
| 红色粗线 | 最终物理椭圆 | 融合后的新障碍物 |
| 绿色 | 反应边界 | 触发融合的关键边界 |
| 橙色 | 排斥边界 | 斥力计算边界 |
| 青色 | 虚拟边界 | 可选的虚拟反应边界 |

## 参数调整

编辑 launch 文件或通过 rosparam:

```yaml
# 关键参数：反应边界偏移（决定融合触发距离）
reactive_offset: 1.0   # 默认值，单位：米

# 其他边界
repulsive_offset: 0.6
virtual_reactive_offset: 1.5
enable_virtual_boundary: false
```

## 验证融合效果

### 1. 检查 ROS 日志
```bash
rostopic echo /rosout | grep "Obstacle Merging"
```
输出示例：
```
Obstacle Merging: 5 -> 3 (merged 1 clusters)
```
- `5`: 输入障碍物数量
- `3`: 融合后数量
- `1 clusters`: 发生了 1 次融合

### 2. 观察可视化窗口
- 灰色虚线显示被删除的原始障碍物
- 红色粗线显示新生成的融合障碍物
- 左上角显示融合统计

### 3. 检查障碍物话题
```bash
rostopic echo /obs_fitting/ellipse_equations
```

## 常见场景

### 场景 1: 两个近距离障碍物
- **触发**: 反应边界 OBB 重叠
- **结果**: 生成 1 个新椭圆覆盖两者

### 场景 2: 链式重叠 (A-B-C)
- **触发**: A 与 B 重叠，B 与 C 重叠
- **结果**: 三者聚为一簇，生成 1 个大椭圆

### 场景 3: 独立障碍物
- **无触发**: 反应边界未重叠
- **结果**: 保持原样输出

## 性能指标
- **延迟**: < 2ms (10 个障碍物)
- **内存**: 可忽略
- **稳定性**: 无异常斥力峰值

## 故障排除

### 问题: 窗口不显示
```bash
export DISPLAY=:0
```

### 问题: 融合过度
降低 `reactive_offset`:
```yaml
reactive_offset: 0.8  # 从 1.0 减小
```

### 问题: 融合不足
增大 `reactive_offset`:
```yaml
reactive_offset: 1.2  # 从 1.0 增大
```

## 技术细节

详细文档: [OBSTACLE_MERGING_GUIDE.md](OBSTACLE_MERGING_GUIDE.md)

## 关键代码位置
- **头文件**: `src/racecar_sim-noetic/bringup/include/bringup/vector_field_planner.h`
- **融合实现**: `src/racecar_sim-noetic/bringup/src/vector_field_planner.cpp`
- **可视化**: `src/racecar_sim-noetic/bringup/src/vector_field_planner_viz.cpp`
- **CMake**: `src/racecar_sim-noetic/bringup/CMakeLists.txt` (已添加 OpenCV 依赖)
