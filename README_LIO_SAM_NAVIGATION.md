# LIO-SAM实时定位建图 + Move_Base路径规划 使用说明

## 项目概述

本项目实现了阿克曼小车在Gazebo仿真环境中使用LIO-SAM进行实时SLAM建图与定位，并结合move_base进行自主路径规划与导航。

## 系统架构

### 1. 核心组件
- **Gazebo仿真**: 提供小车物理仿真和传感器数据（激光雷达、IMU）
- **LIO-SAM**: 实时激光惯性里程计SLAM，提供定位和地图构建
- **Move_Base**: ROS导航栈，提供路径规划和避障
- **TEB局部规划器**: 适配阿克曼模型的局部路径规划

### 2. 坐标系配置

整个系统的坐标系架构如下：

```
map (全局地图坐标系)
  └─ odom (里程计坐标系)
      └─ base_link (机器人主体坐标系)
          └─ base_footprint (地面投影坐标系)
              └─ 传感器frames (velodyne, imu等)
```

#### 关键坐标系说明：

1. **Racecar坐标系** (由`racecar.launch`中的robot_state_publisher发布)
   - `base_footprint`: 机器人在地面的投影坐标系
   - `base_link`: 机器人主体坐标系
   - 传感器坐标系: `velodyne`, `imu`, 各车轮等

2. **LIO-SAM坐标系** (已修改`params.yaml`配置)
   - `mapFrame: "map"` - 全局地图坐标系
   - `odometryFrame: "odom"` - 里程计坐标系
   - `baselinkFrame: "base_link"` - 与racecar的base_link一致
   - `lidarFrame: "base_link"` - 激光数据转换到base_link
   - **LIO-SAM发布TF**: `map -> odom -> base_link`

3. **Navigation坐标系**
   - Global Costmap: `global_frame: map`, `robot_base_frame: base_footprint`
   - Local Costmap: `global_frame: odom`, `robot_base_frame: base_footprint`
   - Move_Base需要完整的TF链路

## 文件说明

### 新增/修改的文件

#### 1. Launch文件
- **`lio_sam_navigation.launch`**: 主启动文件，集成所有模块
  - 启动Gazebo仿真环境
  - 启动LIO-SAM SLAM
  - 启动Move_Base导航
  - 配置所有坐标系

#### 2. 参数配置文件

**LIO-SAM配置** (`src/LIO-SAM/config/params.yaml`)
- 修改了坐标系配置，使其与racecar一致：
  ```yaml
  lidarFrame: "base_link"
  baselinkFrame: "base_link"
  odometryFrame: "odom"
  mapFrame: "map"
  ```

**Costmap配置** (新增文件):
- `costmap_common_params_lio.yaml`: 
  - 使用LIO-SAM的点云数据作为障碍物源
  - 主题: `/lio_sam/mapping/cloud_registered`
  - 配置障碍物检测范围和参数

- `global_costmap_params_lio.yaml`:
  - 全局代价地图配置
  - 使用rolling window模式（动态地图）
  - 不使用静态地图

- `local_costmap_params_lio.yaml`:
  - 局部代价地图配置
  - 用于局部避障和路径规划

## 使用方法

### 1. 启动完整系统

```bash
cd /home/gcs/kitti_test
source devel/setup.bash
roslaunch bringup lio_sam_navigation.launch
```

这个命令会自动启动：
- Gazebo仿真环境和小车模型
- LIO-SAM实时建图与定位
- Move_Base导航系统
- RViz可视化界面

### 2. 使用导航功能

在RViz中：

1. **设置初始位姿（如果需要）**:
   - 点击RViz工具栏的 "2D Pose Estimate"
   - 在地图上点击并拖动设置机器人初始位置和方向

2. **设置导航目标**:
   - 点击RViz工具栏的 "2D Nav Goal"
   - 在地图上点击并拖动设置目标位置和方向
   - 小车会自动规划路径并导航到目标点

3. **观察实时建图**:
   - LIO-SAM会实时构建点云地图
   - 可以在RViz中看到 "LIO-SAM Map" 显示

### 3. 单独启动各模块（调试用）

如果需要单独测试某个模块：

**仅启动Gazebo和小车**:
```bash
roslaunch bringup racecar_gazebo_rviz.launch
```

**仅启动LIO-SAM**:
```bash
roslaunch lio_sam run.launch
```

**仅启动导航（需要先有定位和地图）**:
```bash
roslaunch bringup move_base.launch
```

## 重要话题（Topics）

### LIO-SAM相关
- `/lio_sam/mapping/odometry` - LIO-SAM定位结果
- `/lio_sam/mapping/cloud_registered` - 配准后的点云地图
- `/lio_sam/mapping/path` - SLAM轨迹

### 导航相关
- `/move_base/goal` - 导航目标
- `/move_base/global_costmap/costmap` - 全局代价地图
- `/move_base/local_costmap/costmap` - 局部代价地图
- `/cmd_vel` - 速度控制命令

### 传感器相关
- `/velodyne_points` - 原始激光雷达点云
- `/imu_data` - IMU数据
- `/scan` - 2D激光扫描数据

## TF树结构

完整的TF树应该是：

```
map
└── odom (由LIO-SAM发布)
    └── base_link (由LIO-SAM发布)
        └── base_footprint (由robot_state_publisher发布)
            ├── chassis
            ├── velodyne
            ├── imu
            └── 其他传感器和车轮
```

可以使用以下命令查看TF树：
```bash
rosrun tf view_frames
evince frames.pdf
```

或实时查看：
```bash
rosrun rqt_tf_tree rqt_tf_tree
```

## 参数调整建议

### 1. 导航速度调整

修改 `teb_local_planner_params.yaml`:
```yaml
max_vel_x: 0.5  # 最大线速度
max_vel_theta: 0.5  # 最大角速度
acc_lim_x: 0.5  # 线加速度限制
```

### 2. 障碍物检测调整

修改 `costmap_common_params_lio.yaml`:
```yaml
obstacle_range: 10.0  # 障碍物检测范围
inflation_radius: 0.3  # 障碍物膨胀半径
robot_radius: 0.25  # 机器人半径
```

### 3. LIO-SAM建图质量调整

修改 `src/LIO-SAM/config/params.yaml`:
```yaml
mappingSurfLeafSize: 0.2  # 降低以提高地图精度（但会增加计算量）
loopClosureEnableFlag: true  # 启用回环检测以提高长期一致性
```

## 常见问题

### 1. TF报错: "Transform timeout"
- 检查LIO-SAM是否正常运行
- 确认 `use_sim_time` 参数设置正确
- 查看是否有TF冲突（两个节点发布同一个TF）

### 2. 小车不移动
- 检查 `nav_sim.py` 节点是否运行
- 查看 `/cmd_vel` 话题是否有数据
- 确认Gazebo中的小车控制器正常工作

### 3. 地图质量差
- 调整LIO-SAM的滤波参数
- 确认IMU和激光雷达的外参标定正确
- 降低 `downsampleRate` 以使用更多点云数据

### 4. 导航规划失败
- 增加costmap的 `obstacle_range`
- 调整 `inflation_radius` 避免过度膨胀
- 检查目标点是否在可达区域

## 性能优化

### 1. 计算资源优化
```yaml
# LIO-SAM config
numberOfCores: 4  # 根据CPU核心数调整
mappingProcessInterval: 0.15  # 增加间隔以降低计算负载
```

### 2. 地图更新频率
```yaml
# Global costmap
update_frequency: 2.0  # 降低以减少计算
publish_frequency: 1.0
```

## 扩展功能

### 1. 添加动态障碍物检测
可以添加额外的传感器数据源到costmap

### 2. 保存地图
LIO-SAM可以保存PCD格式地图：
```yaml
savePCD: true
savePCDDirectory: "/your/path/LOAM/"
```

### 3. 多目标点导航
可以使用 `move_base_msgs/MoveBaseAction` 实现waypoint导航

## 作者和维护

配置日期：2025-11-30
ROS版本：Noetic
Gazebo版本：11

## 参考资料
- [LIO-SAM GitHub](https://github.com/TixiaoShan/LIO-SAM)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [TEB Local Planner](http://wiki.ros.org/teb_local_planner)
