# 向量场规划器可视化话题订阅指南

## 📡 目标路径可视化话题

向量场规划器发布了多个可视化话题，用于在RViz中显示路径、向量场和调试信息。

### 🎯 主要路径相关话题

#### 1. **目标路径可视化** (推荐)
```bash
话题名称: /vector_field_planner/circle_path
消息类型: visualization_msgs/Marker
作用: 显示目标路径（圆形/线段/8字形）
颜色: 红色
```

**订阅方法**:
```bash
# 命令行查看
rostopic echo /vector_field_planner/circle_path

# 查看话题信息
rostopic info /vector_field_planner/circle_path

# 查看消息类型
rosmsg show visualization_msgs/Marker
```

**Python订阅示例**:
```python
#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def path_callback(msg):
    print(f"收到路径可视化:")
    print(f"  类型: {msg.type}")
    print(f"  命名空间: {msg.ns}")
    print(f"  点数: {len(msg.points)}")
    if msg.points:
        print(f"  第一个点: ({msg.points[0].x:.2f}, {msg.points[0].y:.2f})")

rospy.init_node('path_subscriber')
rospy.Subscriber('/vector_field_planner/circle_path', Marker, path_callback)
rospy.spin()
```

**C++订阅示例**:
```cpp
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

void pathCallback(const visualization_msgs::Marker::ConstPtr& msg) {
    ROS_INFO("收到路径可视化: %s, 点数=%lu", 
             msg->ns.c_str(), msg->points.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe(
        "/vector_field_planner/circle_path", 1, pathCallback);
    
    ros::spin();
    return 0;
}
```

#### 2. **局部规划路径**
```bash
话题名称: /vector_field_planner/local_plan
消息类型: nav_msgs/Path
作用: 显示车辆当前位置沿向量场积分生成的局部路径
长度: 参数 local_path_length (默认3米)
```

**订阅方法**:
```bash
rostopic echo /vector_field_planner/local_plan
```

**Python订阅示例**:
```python
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path

def local_path_callback(msg):
    print(f"局部路径点数: {len(msg.poses)}")
    if msg.poses:
        first = msg.poses[0].pose.position
        last = msg.poses[-1].pose.position
        print(f"  起点: ({first.x:.2f}, {first.y:.2f})")
        print(f"  终点: ({last.x:.2f}, {last.y:.2f})")

rospy.init_node('local_path_subscriber')
rospy.Subscriber('/vector_field_planner/local_plan', Path, local_path_callback)
rospy.spin()
```

---

## 🧭 向量场可视化话题

### 3. **复合向量场** (橙色箭头)
```bash
话题名称: /vector_field_planner/vector_field
消息类型: visualization_msgs/MarkerArray
作用: 显示路径跟踪+避障的复合向量场
颜色: 橙色
```

### 4. **路径跟踪向量场** (绿色箭头)
```bash
话题名称: /vector_field_planner/path_field
消息类型: visualization_msgs/MarkerArray
作用: 显示纯路径跟踪向量场分量
颜色: 绿色
```

### 5. **避障反应向量场** (红色箭头)
```bash
话题名称: /vector_field_planner/reactive_field
消息类型: visualization_msgs/MarkerArray
作用: 显示障碍物避障向量场分量
颜色: 红色
```

**订阅MarkerArray示例**:
```python
#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray

def vector_field_callback(msg):
    print(f"向量场箭头数量: {len(msg.markers)}")
    for marker in msg.markers[:3]:  # 显示前3个
        if marker.points:
            start = marker.points[0]
            end = marker.points[1]
            print(f"  箭头: ({start.x:.2f},{start.y:.2f}) -> ({end.x:.2f},{end.y:.2f})")

rospy.init_node('vector_field_subscriber')
rospy.Subscriber('/vector_field_planner/vector_field', 
                 MarkerArray, vector_field_callback)
rospy.spin()
```

---

## 🚗 车辆状态可视化

### 6. **当前控制向量** (紫色箭头)
```bash
话题名称: /vector_field_planner/current_vector
消息类型: visualization_msgs/Marker
作用: 显示车辆当前位置的控制向量
颜色: 紫色/品红
长度: 1.5米固定长度（归一化方向）
```

### 7. **偏航角误差**
```bash
话题名称: /vector_field_planner/yaw_error
消息类型: std_msgs/Float32
作用: 发布当前偏航角误差（弧度）
```

**订阅示例**:
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def yaw_error_callback(msg):
    error_deg = msg.data * 180.0 / 3.14159
    print(f"偏航角误差: {error_deg:.2f}度")

rospy.init_node('yaw_error_subscriber')
rospy.Subscriber('/vector_field_planner/yaw_error', Float32, yaw_error_callback)
rospy.spin()
```

---

## 🚧 障碍物可视化

### 8. **障碍物边界**
```bash
话题名称: /vector_field_planner/obstacle_boundaries
消息类型: visualization_msgs/MarkerArray
作用: 显示障碍物的三层边界
  - 原始椭圆 (黄色，透明度0.3)
  - 排斥边界 (红色，透明度0.2)
  - 反应边界 (橙色，透明度0.15)
```

---

## ⏱️ 性能监控话题

### 9. **向量场计算时间**
```bash
话题名称: /vector_field_planner/field_compute_time
消息类型: std_msgs/Float32
单位: 毫秒
```

### 10. **控制查询时间**
```bash
话题名称: /vector_field_planner/control_query_time
消息类型: std_msgs/Float32
单位: 毫秒
```

### 11. **可视化时间**
```bash
话题名称: /vector_field_planner/viz_time
消息类型: std_msgs/Float32
单位: 毫秒
```

### 12. **各步骤时间详情**
```bash
话题名称: /vector_field_planner/step_times
消息类型: std_msgs/Float32MultiArray
单位: 毫秒
数据索引:
  [0]: 向量场计算时间
  [1]: 局部路径生成时间
  [2]: 控制向量获取时间
  [3]: 控制命令转换时间
  [4]: 命令发布时间
  [5]: 可视化时间
  [6]: 总时间
```

**订阅示例**:
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray

def step_times_callback(msg):
    names = ['向量场计算', '局部路径', '控制查询', '命令转换', 
             '命令发布', '可视化', '总时间']
    print("执行时间 (ms):")
    for i, (name, time) in enumerate(zip(names, msg.data)):
        print(f"  {name}: {time:.2f}")

rospy.init_node('performance_monitor')
rospy.Subscriber('/vector_field_planner/step_times', 
                 Float32MultiArray, step_times_callback)
rospy.spin()
```

---

## 🎨 在RViz中添加可视化

### 快速配置

1. **启动RViz**:
```bash
rviz
```

2. **添加目标路径**:
   - 点击 "Add" → "By topic"
   - 选择 `/vector_field_planner/circle_path` → `Marker`
   - 路径会显示为红色线条

3. **添加局部路径**:
   - 选择 `/vector_field_planner/local_plan` → `Path`
   - 建议颜色：蓝色或青色

4. **添加向量场**:
   - 选择 `/vector_field_planner/vector_field` → `MarkerArray`
   - 复合向量场（橙色）

5. **添加当前控制向量**:
   - 选择 `/vector_field_planner/current_vector` → `Marker`
   - 紫色箭头

6. **添加障碍物边界**:
   - 选择 `/vector_field_planner/obstacle_boundaries` → `MarkerArray`

### RViz配置文件

查看现有配置文件：
```bash
# LIO-SAM导航配置
cat slam_nav.rviz

# 简单配置
cat lio_sam_simple.rviz
```

---

## 📊 完整话题列表

```bash
# 查看所有规划器发布的话题
rostopic list | grep vector_field_planner

# 输出示例:
/vector_field_planner/circle_path              # 目标路径 ⭐
/vector_field_planner/local_plan               # 局部路径 ⭐
/vector_field_planner/vector_field             # 复合向量场
/vector_field_planner/path_field               # 路径向量场
/vector_field_planner/reactive_field           # 避障向量场
/vector_field_planner/current_vector           # 当前控制向量
/vector_field_planner/obstacle_boundaries      # 障碍物边界
/vector_field_planner/yaw_error                # 偏航误差
/vector_field_planner/field_compute_time       # 计算时间
/vector_field_planner/control_query_time       # 查询时间
/vector_field_planner/viz_time                 # 可视化时间
/vector_field_planner/step_times               # 步骤时间
```

---

## 🔧 实用命令

### 查看话题频率
```bash
rostopic hz /vector_field_planner/circle_path
```

### 记录所有可视化数据
```bash
rosbag record -O path_viz.bag \
  /vector_field_planner/circle_path \
  /vector_field_planner/local_plan \
  /vector_field_planner/vector_field
```

### 回放记录数据
```bash
rosbag play path_viz.bag
```

### 实时监控性能
```bash
# 监控计算时间
rostopic echo /vector_field_planner/field_compute_time

# 监控所有步骤时间
rostopic echo /vector_field_planner/step_times
```

---

## 💡 常见使用场景

### 场景1: 只看目标路径和车辆轨迹
```bash
# RViz中添加:
1. /vector_field_planner/circle_path (目标路径)
2. /vector_field_planner/local_plan (预测轨迹)
3. /lio_sam/mapping/odometry (车辆位置)
```

### 场景2: 调试向量场
```bash
# RViz中添加:
1. /vector_field_planner/path_field (绿色-路径跟踪)
2. /vector_field_planner/reactive_field (红色-避障)
3. /vector_field_planner/vector_field (橙色-复合)
4. /vector_field_planner/current_vector (紫色-当前)
```

### 场景3: 监控避障行为
```bash
# RViz中添加:
1. /vector_field_planner/obstacle_boundaries
2. /vector_field_planner/reactive_field
3. /obs_fitting/ellipse_equations (原始障碍物)
```

### 场景4: 性能分析
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray

class PerformanceMonitor:
    def __init__(self):
        rospy.init_node('perf_monitor')
        rospy.Subscriber('/vector_field_planner/step_times', 
                        Float32MultiArray, self.callback)
        self.times = []
    
    def callback(self, msg):
        self.times.append(msg.data[6])  # 总时间
        if len(self.times) >= 100:
            avg_time = sum(self.times) / len(self.times)
            max_time = max(self.times)
            print(f"平均: {avg_time:.2f}ms, 最大: {max_time:.2f}ms")
            self.times = []
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    monitor = PerformanceMonitor()
    monitor.run()
```

---

## 📝 注意事项

1. **话题名称统一**: 所有路径可视化都通过 `/vector_field_planner/circle_path` 发布，无论是圆形、线段还是8字形路径

2. **坐标系**: 所有可视化使用 `map` 坐标系

3. **频率控制**: 
   - 路径可视化：启动时发布一次（latch=true）
   - 向量场：由 `viz_frequency` 参数控制（默认10Hz）
   - 局部路径：实时更新（控制频率）

4. **性能影响**: 
   - 向量场可视化会消耗计算资源
   - 可通过降低 `field_resolution` 或 `viz_frequency` 优化

5. **消息延迟**: 
   - 可视化消息可能有轻微延迟
   - 不影响实际控制（控制独立于可视化）

---

## 🚀 快速开始

```bash
# 1. 启动规划器
roslaunch bringup circular_path_navigation_cpp.launch

# 2. 查看所有话题
rostopic list | grep vector_field_planner

# 3. 订阅目标路径
rostopic echo /vector_field_planner/circle_path

# 4. 在RViz中可视化
rviz
# 然后添加 /vector_field_planner/circle_path
```

现在你可以轻松订阅和可视化所有路径相关的信息了！🎉
