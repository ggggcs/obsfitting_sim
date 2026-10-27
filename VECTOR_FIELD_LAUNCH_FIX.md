# 向量场导航启动问题修复

## 问题描述
启动 `vector_field_navigation.launch` 后出现以下问题：
1. 在RViz中设置目标点，车辆一直等待全局路径
2. 坐标系错误：Unknown frame map
3. TF错误：No transform from [base_footprint] to [map]

## 根本原因

1. **话题订阅错误**：向量场规划器订阅 `/move_base/GlobalPlanner/plan`，但move_base实际发布 `/move_base/NavfnROS/plan`

2. **controller_frequency设为0**：这会导致move_base的控制循环完全停止，从而无法触发全局路径规划

3. **缺少TF转换节点**：launch文件中没有启动必要的TF发布节点，导致坐标系树不完整

## 解决方案

### 1. 修复话题订阅

**文件**: `src/racecar_sim-noetic/bringup/scripts/vector_field_planner.py`

```python
# 修改前
rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_path_callback)

# 修改后
rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.global_path_callback)
```

### 2. 修复controller_frequency

**文件**: `src/racecar_sim-noetic/bringup/launch/vector_field_navigation.launch`

```xml
<!-- 修改前 -->
<param name="controller_frequency" value="0.0"/>

<!-- 修改后 -->
<param name="controller_frequency" value="5.0"/>
```

**说明**: move_base需要控制循环运行才能触发全局规划器。即使我们不使用move_base的局部规划器，也需要保持一定的更新频率。

### 3. 添加TF转换节点

**文件**: `src/racecar_sim-noetic/bringup/launch/vector_field_navigation.launch`

在LIO-SAM启动后添加以下节点：

```xml
<!-- 里程计消息中继节点 -->
<node name="odom_relay" pkg="bringup" type="odom_relay.py" output="screen"/>

<!-- 点云转激光扫描 -->
<node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
    <remap from="cloud_in" to="/lio_sam/mapping/cloud_registered"/>
    <remap from="scan" to="/scan"/>
    <!-- 参数配置... -->
</node>

<!-- 机器人状态发布器 -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
    <param name="publish_frequency" value="50.0"/>
</node>

<!-- 静态TF转换 -->
<node pkg="tf" type="static_transform_publisher" name="base_link_lio_to_base_link"
      args="0 0 0 0 0 0 base_link_lio base_link 100"/>

<node pkg="tf" type="static_transform_publisher" name="base_link_to_base_footprint"
      args="0 0 0.05 0 0 0 base_link base_footprint 100"/>
```

**说明**: 这些节点确保TF树完整：`map -> odom -> base_footprint -> base_link -> velodyne`

## 重启系统

```bash
# 1. 终止当前所有相关进程
# 在终端按 Ctrl+C 停止所有节点

# 2. 重新启动
roslaunch bringup vector_field_navigation.launch
```

## 验证步骤

1. **检查TF树是否完整**:
```bash
rosrun tf view_frames
# 或者
rosrun rqt_tf_tree rqt_tf_tree
```
应该看到完整的TF链：`map -> odom -> base_footprint -> base_link`

2. **检查关键TF变换**:
```bash
rosrun tf tf_echo map base_footprint
```
应该能看到变换数据，而不是错误

3. **检查向量场规划器是否正确订阅**:
```bash
rosnode info /vector_field_planner | grep Subscriptions -A 5
```
应该看到订阅 `/move_base/NavfnROS/plan`

4. **检查全局路径是否发布**:
```bash
rostopic hz /move_base/NavfnROS/plan
```
设置目标后应该看到路径发布

5. **手动测试发送目标**:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' -1
```

## 预期行为

- 启动后车辆静止，终端显示"等待全局路径..."
- 在RViz使用"2D Nav Goal"设置目标点
- move_base发布全局路径到 `/move_base/NavfnROS/plan`
- 向量场规划器接收路径并开始控制车辆
- 终端显示路径信息："收到新的全局路径,包含 XX 个点"

## 注意事项

- move_base的`cmd_vel`已重映射到`move_base/cmd_vel`，不会与向量场规划器冲突
- 实际控制命令由向量场规划器发布到`/cmd_vel`
- move_base仅用于全局路径规划，不执行局部规划和控制
