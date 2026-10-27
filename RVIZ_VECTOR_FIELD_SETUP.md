# RViz 向量场可视化配置指南

## 功能说明

向量场规划器现在会在局部地图范围内发布三种向量场的可视化:

1. **复合向量场** (橙色箭头) - `/vector_field_planner/vector_field`
   - 最终用于控制的综合向量场
   - χ_c = (∏U_Q)χ_P + ∑(T_R χ_R)
   - 颜色: RGB(1.0, 0.5, 0.0)
   - 高度: z=0.1m

2. **路径跟踪向量场** (绿色箭头) - `/vector_field_planner/path_field`
   - 指向全局路径的引导向量
   - 归一化单位向量
   - 颜色: RGB(0.0, 1.0, 0.0)
   - 高度: z=0.05m

3. **反应避障向量场** (红色箭头) - `/vector_field_planner/reactive_field`
   - 从障碍物椭圆边界产生的斥力向量
   - 归一化单位向量
   - 颜色: RGB(1.0, 0.0, 0.0)
   - 高度: z=0.15m

## 可视化参数

在 `vector_field_planner_params.yaml` 中配置:

```yaml
viz_range: 8.0          # 可视化范围 (米), 以当前位置为中心的正方形区域
viz_resolution: 0.5     # 网格分辨率 (米), 决定箭头密度
viz_frequency: 2.0      # 更新频率 (Hz)
```

**性能说明:**
- `viz_range=8.0, viz_resolution=0.5` → 约17x17=289个网格点
- 更新频率2Hz, 每次计算约300次向量场
- 如果性能不足, 可增大resolution或减小range

## RViz 配置步骤

### 1. 添加复合向量场显示

1. 点击 RViz 左下角 "Add" 按钮
2. 选择 "By topic" 标签
3. 找到 `/vector_field_planner/vector_field`
4. 选择 `Marker` 类型, 点击 OK
5. 在左侧面板中设置:
   - ☑ Enabled
   - Topic: `/vector_field_planner/vector_field`
   - 其他保持默认

### 2. 添加路径跟踪向量场显示

1. 点击 "Add" 按钮
2. 选择 "By topic"
3. 找到 `/vector_field_planner/path_field`
4. 选择 `Marker`, 点击 OK

### 3. 添加反应避障向量场显示

1. 点击 "Add" 按钮
2. 选择 "By topic"
3. 找到 `/vector_field_planner/reactive_field`
4. 选择 `Marker`, 点击 OK

### 4. 调整显示效果

在每个Marker显示项中可以调整:
- **Namespaces**: 勾选对应的命名空间
- **Color**: 如需修改颜色 (默认已设置)
- **Scale**: 如果箭头太小/太大

## 使用方法

### 启动系统

```bash
# 启动向量场导航
roslaunch bringup vector_field_navigation.launch
```

### 打开RViz

```bash
# 使用导航配置文件
rviz -d ~/kitti_test/slam_nav.rviz
```

或在RViz中手动添加上述Marker显示项。

### 给定导航目标

在RViz中使用 "2D Nav Goal" 工具设置目标点, 观察:

1. **绿色箭头** (路径场) - 应该指向全局路径方向
2. **红色箭头** (反应场) - 在障碍物附近出现, 指向远离方向
3. **橙色箭头** (复合场) - 综合效果, 实际控制依据

### 调试观察

- 检查箭头方向是否合理
- 观察避障时向量场的变化
- 注意障碍物附近的反应场激活情况
- 验证路径跟踪的平滑性

## 向量场计算细节

### χ_P (路径跟踪向量场)

- 查找最近的 `lookahead=5` 个路径点
- 计算从当前位置到该点的单位向量
- 归一化: ||χ_P|| = 1

### χ_R (反应避障向量场)

对每个椭圆障碍物:
1. 计算椭圆函数 φ(x,y) 和梯度 ∇φ
2. 如果 φ < 0 (在椭圆内), 沿梯度方向逃逸
3. 归一化: ||χ_R|| = 1

### χ_c (复合向量场)

```
χ_c = (∏U_Q) · χ_P + ∑(T_R · χ_R)
```

其中:
- U_Q: 路径跟踪权重 (距离障碍物越近越小)
- T_R: 反应避障权重 (距离障碍物越近越大)

## 故障排查

### 看不到箭头

1. 检查话题是否发布:
   ```bash
   rostopic hz /vector_field_planner/vector_field
   rostopic hz /vector_field_planner/path_field
   rostopic hz /vector_field_planner/reactive_field
   ```

2. 检查Marker消息内容:
   ```bash
   rostopic echo /vector_field_planner/vector_field
   ```

3. 确认RViz的Fixed Frame设置为 `odom`

### 箭头太密集/稀疏

修改 `viz_resolution`:
- 增大 (如0.8) → 箭头更稀疏
- 减小 (如0.3) → 箭头更密集

### 可视化卡顿

1. 降低更新频率: `viz_frequency: 1.0`
2. 减小可视化范围: `viz_range: 5.0`
3. 增大网格分辨率: `viz_resolution: 0.8`

### 箭头看不清

箭头在不同高度:
- 复合场: z=0.1m (中间)
- 路径场: z=0.05m (底层)
- 反应场: z=0.15m (顶层)

可以调整RViz的视角俯视观察。

## 参数调优建议

### 性能优先

```yaml
viz_range: 5.0
viz_resolution: 0.8
viz_frequency: 1.0
```

### 细节优先

```yaml
viz_range: 10.0
viz_resolution: 0.3
viz_frequency: 3.0
```

### 平衡配置 (推荐)

```yaml
viz_range: 8.0
viz_resolution: 0.5
viz_frequency: 2.0
```

## 高级用法

### 动态调整可视化参数

```bash
# 修改可视化范围
rosparam set /vector_field_planner/viz_range 10.0

# 修改网格分辨率
rosparam set /vector_field_planner/viz_resolution 0.3

# 修改更新频率
rosparam set /vector_field_planner/viz_frequency 1.0
```

**注意**: 需要重启节点才能生效。

### 只显示特定向量场

在RViz中取消勾选不需要的Marker显示项即可。

### 录制向量场视频

```bash
# 录制所有话题
rosbag record -a

# 只录制向量场相关话题
rosbag record /vector_field_planner/vector_field \
              /vector_field_planner/path_field \
              /vector_field_planner/reactive_field \
              /obs_fitting/ellipse_equations \
              /move_base/GlobalPlanner/plan
```

## 相关文件

- **源代码**: `src/racecar_sim-noetic/bringup/scripts/vector_field_planner.py`
- **参数文件**: `src/racecar_sim-noetic/bringup/param/vector_field_planner_params.yaml`
- **启动文件**: `src/racecar_sim-noetic/bringup/launch/vector_field_navigation.launch`
- **主文档**: `VECTOR_FIELD_INTEGRATION.md`

## 下一步

1. 启动系统并在RViz中添加向量场显示
2. 给定导航目标观察向量场分布
3. 根据实际效果调整可视化参数
4. 如需调整控制参数, 参考 `VECTOR_FIELD_INTEGRATION.md`
