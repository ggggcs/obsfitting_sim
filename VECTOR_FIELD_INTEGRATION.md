# 向量场导航系统集成指南

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    向量场导航系统                              │
└─────────────────────────────────────────────────────────────┘

1. 全局规划层 (Global Planning)
   ├─ NavFn (A*算法)
   ├─ 输入: LIO-SAM全局代价地图
   └─ 输出: 全局路径

2. 局部规划层 (Local Planning - 向量场方法)
   ├─ 路径跟踪向量场 χ_P
   │  └─ 跟踪全局路径
   ├─ 障碍物反应向量场 χ_R  
   │  └─ 基于obs_fitting椭圆实时避障
   └─ 复合向量场 χ_c = (∏U_Q)χ_P + ∑(T_R χ_R)

3. 控制层
   └─ Ackermann控制转换
```

## 核心文件

### 1. 向量场规划器
**文件**: `scripts/vector_field_planner.py`
**功能**: 
- 订阅全局路径和障碍物椭圆
- 计算复合向量场
- 发布速度命令

**关键参数**:
```yaml
k_p: 1.0              # 路径跟踪增益
k_r: 1.2              # 反应向量场增益
repulsive_offset: 0.4 # 排斥边界
reactive_offset: 0.8  # 反应边界
```

### 2. 消息接口

**直接使用obs_fitting已有的话题**:
```python
# 订阅话题
/obs_fitting/ellipse_equations  # 类型: std_msgs/String

# 消息格式示例:
"""
检测到 2 个椭圆:
椭圆 ID:0 中心:(1.234,-2.567) 大小:2.000x1.500 角度:45.000° 面积:2.356
椭圆 ID:0 方程: 0.625000x2 + -0.500000xy + 0.875000y2 + -1.550000x + 2.834000y + -0.200000 = 0
---
椭圆 ID:1 中心:(3.456,1.789) 大小:1.800x1.200 角度:-30.000° 面积:1.696
椭圆 ID:1 方程: 0.750000x2 + 0.400000xy + 0.650000y2 + -5.200000x + -1.500000y + 3.800000 = 0
---
"""
```

### 3. 与obs_fitting集成

**obs_fitting已经实现**:
- ✅ 发布话题: `/obs_fitting/ellipse_equations`
- ✅ 消息类型: `std_msgs/String`
- ✅ 包含: ID, 中心坐标, 尺寸, 角度, 方程系数
- ✅ 坐标系: `hesai_lidar` (已在代码中处理坐标转换)

**vector_field_planner已实现**:
- ✅ 订阅并解析椭圆方程字符串
- ✅ 提取椭圆参数 (center, size, angle)
- ✅ 转换为EllipseObstacle对象用于向量场计算

## 使用步骤

### 1. 无需额外消息定义
obs_fitting已经发布所需的椭圆信息,无需创建新的消息类型。

### 2. 编译
```bash
cd ~/kitti_test
catkin_make
source devel/setup.bash
```

### 3. 启动系统
```bash
# 使用向量场导航
roslaunch bringup vector_field_navigation.launch

# 或继续使用TEB (原系统)
roslaunch bringup lio_sam_navigation.launch
```

### 4. 测试
```bash
# 查看向量场规划器状态
rostopic echo /vector_field_planner/local_plan

# 查看障碍物椭圆方程
rostopic echo /obs_fitting/ellipse_equations

# 查看obs_fitting可视化
rostopic list | grep obs_fitting
# /obs_fitting/ellipses (Marker可视化)
# /obs_fitting/ellipse_equations (String方程信息)

# 在RViz中发送导航目标
# 使用"2D Nav Goal"工具点击地图
```

## 参数调优

### 向量场参数
```yaml
# 增加路径跟踪精度
k_p: 1.5  # 提高 -> 更紧贴路径

# 增加避障反应
k_r: 2.0  # 提高 -> 更强避障

# 调整安全边界
repulsive_offset: 0.3   # 减小 -> 更激进
reactive_offset: 0.6    # 减小 -> 更激进

# 调整速度
max_speed: 1.5  # 降低 -> 更安全
```

### Bump函数参数
```yaml
l1: 0.5  # 增大 -> 过渡更平滑
l2: 0.5  # 增大 -> 过渡更平滑
```

## 调试技巧

### 1. 可视化
在RViz中添加:
- `/vector_field_planner/local_plan` (Path)
- `/obs_fitting/ellipses` (Marker)
- `/move_base/GlobalPlanner/plan` (Path)

### 2. 日志
```bash
# 查看规划器日志
rosrun rqt_console rqt_console

# 设置日志级别
rosservice call /vector_field_planner/set_logger_level ros.bringup DEBUG
```

### 3. 常见问题
- **小车不动**: 检查是否收到全局路径和障碍物数据
- **避障过于保守**: 减小`reactive_offset`和`repulsive_offset`
- **路径跟踪不准**: 增大`k_p`
- **震荡**: 减小`k_r`或增大`l1`, `l2`

## 对比TEB规划器

| 特性 | TEB规划器 | 向量场规划器 |
|------|-----------|-------------|
| 算法 | 时间弹性带优化 | 引导向量场 |
| 障碍物表示 | 点云 | 拟合椭圆 |
| 计算复杂度 | 高 (优化求解) | 低 (梯度计算) |
| 平滑性 | 优化保证 | 向量场连续 |
| 动态避障 | 好 | 优秀 |
| 阿克曼约束 | 内置 | 后处理 |
| 参数调优 | 复杂 | 简单直观 |

## 下一步

1. ✅ 在obs_fitting中实现椭圆消息发布
2. ✅ 测试向量场规划器
3. ✅ 对比TEB和向量场性能
4. ✅ 根据实际效果调优参数
5. ✅ 添加RViz可视化标记
