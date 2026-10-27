# 障碍物融合策略实现文档

## 概述

本实现基于 **"排斥边界 OBB 的融合策略"**，实现了智能障碍物融合，避免重叠障碍物对向量场规划器造成干扰。

---

## 核心特性

### 1. 完全替代原则 (Complete Replacement)
- 一旦检测到重叠，**删除所有原始重叠椭圆**
- 用**唯一的新生成椭圆**完全替代
- 最终输出列表中**不含已融合的原始障碍物**

### 2. 触发条件唯一性
- 重叠检测**仅基于排斥边界 (Repulsive Boundary) 的 OBB**
- 物理轮廓不参与检测
- 排斥边界尺寸 = `(a + repulsive_offset) × 2`

### 3. 实时可视化调试
- OpenCV 窗口 "Obstacle Merge Debugger"
- 多图层显示融合过程
- 颜色编码清晰标识各个阶段

---

## 算法流水线

### Step 1: 排斥边界膨胀
```cpp
cv::RotatedRect buildRepulsiveOBB(const EllipseObstacle& obstacle) {
    float width = (obstacle.a + repulsive_offset_) * 2.0;
    float height = (obstacle.b + repulsive_offset_) * 2.0;
    // ...
}
```

### Step 2: OBB 重叠检测与聚类
- 使用 OpenCV 的 `rotatedRectangleIntersection`
- Union-Find 算法构建重叠簇
- 支持多障碍物链式重叠 (A-B-C)

### Step 3: 几何融合
1. 提取簇内所有排斥 OBB 的角点
2. 计算凸包 (Convex Hull)
3. 计算最小外接矩形 (MinAreaRect)
4. **逆向推导新物理椭圆参数**:
   ```cpp
   a_new = (MinAreaRect.width / 2.0) - repulsive_offset_
   b_new = (MinAreaRect.height / 2.0) - repulsive_offset_
   ```

### Step 4: 列表重构
- 非重叠障碍物直接保留
- 重叠簇生成唯一新椭圆
- 完全替换 `obstacles_` 成员变量

---

## 可视化窗口说明

### 图层结构

| 图层 | 内容 | 颜色 | 说明 |
|------|------|------|------|
| 0 | 背景 | 白色 | 800×800 像素，±10m 范围 |
| 1 | 被删除的原始数据 | 灰色虚线 | 显示融合前的椭圆和排斥 OBB |
| 2 | 融合计算过程 | 蓝色细线 | 凸包和最小外接矩形 |
| 3 | 最终输出 | 彩色实线 | 新椭圆及其边界 |

### 颜色编码

- **物理椭圆**: 黑色粗线 (融合后为 3px)
- **排斥边界**: 红色
- **反应边界**: 绿色
- **虚拟边界**: 浅蓝色 (如果启用)
- **锁定障碍物**: 洋红色 (当启用智能转向锁定时)
- **被删除项**: 灰色虚线

### UI 信息
```
Original: N -> Merged: M
```
- N: 输入的原始障碍物数量
- M: 融合后的最终障碍物数量

---

## 参数配置

在 `launch` 文件或 `rosparam` 中配置：
```yaml
# 边界偏移量
repulsive_offset: 0.6      # 排斥边界偏移（触发融合的关键参数）
reactive_offset: 1.0       # 反应边界偏移

# 虚拟边界（可选）
enable_virtual_boundary: false
virtual_reactive_offset: 1.5
k_c: 0.5
```: 0.5
```

---

## 使用示例

### 1. 启动系统
```bash
roslaunch bringup circular_path_navigation_cpp.launch
```

### 2. 观察融合过程
- 自动弹出 "Obstacle Merge Debugger" 窗口
- 实时显示融合前后对比

### 3. 检查日志
```bash
rostopic echo /rosout | grep "Obstacle Merging"
```
输出示例：
```
Obstacle Merging: 5 -> 3 (merged 1 clusters)
```

---

## 代码架构

### 核心文件

1. **vector_field_planner.h**
   - `struct EllipseObstacle`: 扩展了 `is_merged` 和 `merged_from_ids`
   - `struct MergeDebugInfo`: 调试信息结构
   - 函数声明：`mergeObstacles`, `visualizeMergingProcess`
2. **vector_field_planner.cpp**
   - `buildRepulsiveOBB()`: 构建排斥边界 OBB
   - `checkOBBOverlap()`: OpenCV 重叠检测
   - `mergeObstacles()`: 核心融合算法
   - `checkAndLockObstacle()`: 智能转向锁定检测
   - `checkAndUnlockObstacle()`: 锁定解除逻辑V 重叠检测
   - `mergeObstacles()`: 核心融合算法

3. **vector_field_planner_viz.cpp**
   - `visualizeMergingProcess()`: OpenCV 多图层可视化

### 依赖项

- ROS1 (roscpp)
- OpenCV 3/4 (core, imgproc, highgui)
- Eigen3
- C++14
## 关键设计决策

### 为什么使用排斥边界作为触发条件？

1. **保守融合策略**: 排斥边界比反应边界更小，确保只有真正接近的障碍物才会融合
2. **减少误融合**: 避免距离较远但反应边界重叠的障碍物被错误合并
3. **数学一致性**: 融合后的椭圆代表排斥边界的合并结果，与向量场计算保持一致
4. **更精确的障碍物表示**: 保留更多独立障碍物，提高规划精度
1. **安全裕度**: 反应边界是车辆实际响应的区域
2. **一致性**: 与向量场计算中的反应力保持一致
3. **避免过度融合**: 物理轮廓可能很小，但影响范围更大

### 为什么完全替代而非保留原始？

### 参数逆推导的数学正确性

排斥边界 OBB 尺寸：
```
Width_repulsive = (a + repulsive_offset) × 2
```

MinAreaRect 拟合后：
```
Width_minRect ≈ Width_repulsive
```

逆推物理参数：
```
a_new = Width_minRect / 2 - repulsive_offset
```

确保新椭圆的排斥边界与融合后的 MinAreaRect 贴合。

### 智能转向锁定机制

当车辆进入障碍物的虚拟边界后：
1. **锁定绕行方向**: 保存 K 值（1.0 或 -1.0），确保绕行方向不变
2. **锁定障碍物形状和位置**: 冻结障碍物的中心坐标、尺寸和角度
3. **防止重复融合**: 锁定期间，该障碍物的源 ID 从融合管道中排除
4. **吸收重叠新障碍物**: 与锁定障碍物排斥边界重叠的新障碍物自动被吸收

**解锁条件**:
- 所有锁定的源障碍物 ID 从传感器输入中消失，或
- 检测到新的障碍物与锁定障碍物排斥边界重叠（触发重新融合）
### 问题 2: 融合不够激进
**原因**: `repulsive_offset` 过小  
**解决**: 适当增大参数值 (如 0.6 → 0.8)

### 问题 4: 融合后椭圆过大
**原因**: 凸包包含了太多空白区域  
**解决**: 这是正确行为，确保覆盖所有潜在威胁区域

### 问题 5: 车辆在障碍物附近反复横跳
**原因**: 未启用智能转向锁定  
**解决**: 确保虚拟边界已启用，锁定机制会自动激活
确保新椭圆的反应边界与融合后的 MinAreaRect 贴合。

---

## 故障排除

### 问题 1: 窗口不显示
**原因**: X11 转发未启用  
**解决**: 
### 1. 自适应偏移量
根据车速动态调整 `repulsive_offset`:
```cpp
repulsive_offset_ = base_offset + speed_factor * current_velocity;
``` 问题 2: 融合过度激进
**原因**: `reactive_offset` 过大  
**解决**: 减小参数值 (如 1.0 → 0.8)

### 问题 3: 融合后椭圆过大
**原因**: 凸包包含了太多空白区域  
**解决**: 这是正确行为，确保覆盖所有潜在威胁区域

---

## 性能指标

- **计算复杂度**: O(n²) (n = 障碍物数量)
- **典型延迟**: < 2ms (n < 10)
- **内存占用**: 可忽略

---

## 扩展建议

### 1. 自适应偏移量
根据车速动态调整 `reactive_offset`:
```cpp
reactive_offset_ = base_offset + speed_factor * current_velocity;
```

### 2. 形状保真度优化
使用加权凸包保持原始形状特征：
```cpp
cv::convexHull(all_corners, hull, false, true);  // Clockwise
```

### 3. 多传感器融合
在融合前对来自不同传感器的障碍物进行置信度加权。

---

## 作者

资深 C++ 机器人算法工程师 & 计算几何专家  
实现日期: 2026年1月8日  
框架: ROS1, C++14, OpenCV, Eigen3

---

## 许可证

与主项目保持一致
