# 向量场规划器：基于OBB的障碍物融合与智能锁定完整实现指南

## 角色定位
**资深 C++ 机器人算法工程师 & 计算几何专家**

## 技术栈
- **ROS1** (Noetic)
- **C++14/17**
- **OpenCV 3/4** (几何计算、可视化)
- **Eigen3** (向量运算)
- **TF库** (坐标变换)

---

## 任务总览

修改 `vector_field_planner.cpp` 和 `.h` 文件，实现：
1. **基于反应边界 OBB 的障碍物融合策略**
2. **智能转向锁定机制（进入虚拟边界时锁定）**
3. **OpenCV 实时调试窗口（坐标系与 obs_2d 一致）**

---

## 第一部分：障碍物融合算法

### 核心原则（必须严格遵守）

#### 1. 触发条件唯一性
- 重叠检测 **仅基于 "反应边界 (Reactive Boundary)" 的 OBB**
- 物理轮廓 **不参与** 重叠检测
- 反应边界定义：`(a + reactive_offset_) × 2` 和 `(b + reactive_offset_) × 2`

#### 2. 完全替代原则 (Complete Replacement)
- 一旦检测到重叠，**必须删除** 原始的所有重叠椭圆
- 用 **唯一的** 新生成椭圆替代它们
- 最终输出给规划器的列表中，**绝对不能** 出现已被融合的原始障碍物

#### 3. ID 追踪原则
- 每个障碍物有稳定的 `id`（来自上游感知节点）
- 融合后的障碍物需要记录：
  - `is_merged = true`
  - `merged_from_ids` 包含所有源障碍物 ID

---

### 融合算法流水线 (`mergeObstacles` 函数)

#### Step 1: 反应边界膨胀 (Reactive Inflation)

```cpp
cv::RotatedRect VectorFieldPlanner::buildReactiveOBB(const EllipseObstacle& obstacle) {
    // 反应边界尺寸 = 物理尺寸 + reactive_offset
    float width = static_cast<float>((obstacle.a + reactive_offset_) * 2.0);
    float height = static_cast<float>((obstacle.b + reactive_offset_) * 2.0);
    
    cv::Point2f center(static_cast<float>(obstacle.center_x), 
                      static_cast<float>(obstacle.center_y));
    
    // OpenCV RotatedRect angle is in degrees
    float angle_deg = static_cast<float>(obstacle.theta * 180.0 / M_PI);
    
    return cv::RotatedRect(center, cv::Size2f(width, height), angle_deg);
}
```

**关键点：**
- 膨胀量 = `reactive_offset_`（通常 0.3-0.5m）
- 角度从弧度转为度数
- 尺寸是直径（已 ×2）

#### Step 2: 聚类与检测 (Clustering with Union-Find)

**使用 Union-Find 算法进行聚类：**

```cpp
// 初始化并查集
std::vector<int> parent(obstacles.size());
std::iota(parent.begin(), parent.end(), 0);

std::function<int(int)> find = [&](int x) {
    return parent[x] == x ? x : parent[x] = find(parent[x]);
};

auto unite = [&](int x, int y) {
    parent[find(x)] = find(y);
};

// 两两检测重叠并合并
for (size_t i = 0; i < obstacles.size(); ++i) {
    cv::RotatedRect obb_i = buildReactiveOBB(obstacles[i]);
    
    for (size_t j = i + 1; j < obstacles.size(); ++j) {
        cv::RotatedRect obb_j = buildReactiveOBB(obstacles[j]);
        
        // OpenCV 重叠检测
        std::vector<cv::Point2f> intersection_points;
        int result = cv::rotatedRectangleIntersection(obb_i, obb_j, intersection_points);
        
        // INTERSECT_NONE=0, INTERSECT_PARTIAL=1, INTERSECT_FULL=2
        if (result == cv::INTERSECT_PARTIAL || result == cv::INTERSECT_FULL) {
            unite(i, j);  // 合并到同一簇
        }
    }
}

// 构建簇映射
std::map<int, std::vector<int>> clusters;
for (size_t i = 0; i < obstacles.size(); ++i) {
    clusters[find(i)].push_back(i);
}
```

**关键点：**
- 使用 `cv::rotatedRectangleIntersection` 判断 OBB 重叠
- 返回值：0=无交集，1=部分交集，2=完全包含
- Union-Find 保证传递性：A⊕B, B⊕C ⇒ {A,B,C} 在同一簇

#### Step 3: 几何融合与参数重构 (Merge & Derive)

**对每个簇进行融合：**

```cpp
for (const auto& [root, indices] : clusters) {
    if (indices.size() == 1) {
        // 单独障碍物，直接保留
        merged_obstacles.push_back(obstacles[indices[0]]);
        continue;
    }
    
    // === 多个障碍物需要融合 ===
    
    // 1. 收集所有反应 OBB 的角点
    std::vector<cv::Point2f> all_corners;
    std::vector<int> source_ids;
    
    for (int idx : indices) {
        cv::RotatedRect obb = buildReactiveOBB(obstacles[idx]);
        cv::Point2f corners[4];
        obb.points(corners);
        
        for (int k = 0; k < 4; ++k) {
            all_corners.push_back(corners[k]);
        }
        
        source_ids.push_back(obstacles[idx].id);
    }
    
    // 2. 计算凸包
    std::vector<cv::Point2f> convex_hull;
    cv::convexHull(all_corners, convex_hull);
    
    // 3. 计算最小外接矩形
    cv::RotatedRect min_rect = cv::minAreaRect(convex_hull);
    
    // 4. 逆向推导新的物理椭圆参数
    double new_a = (min_rect.size.width / 2.0) - reactive_offset_;
    double new_b = (min_rect.size.height / 2.0) - reactive_offset_;
    
    // 安全检查
    if (new_a <= 0.0) new_a = 0.05;
    if (new_b <= 0.0) new_b = 0.05;
    
    double new_theta = min_rect.angle * M_PI / 180.0;
    
    // 5. 创建新的融合障碍物
    EllipseObstacle merged;
    merged.id = generateNewId();  // 或使用最小 ID
    merged.center_x = min_rect.center.x;
    merged.center_y = min_rect.center.y;
    merged.a = new_a;
    merged.b = new_b;
    merged.theta = new_theta;
    merged.is_merged = true;
    merged.merged_from_ids = source_ids;
    
    merged_obstacles.push_back(merged);
}
```

**关键计算逻辑：**
1. **凸包计算**：确保覆盖所有角点
2. **MinAreaRect**：找到最紧凑的外接矩形
3. **逆向推导**：
   ```
   反应尺寸 = 物理尺寸 + offset
   ⇒ 物理尺寸 = 反应尺寸 - offset
   ```
4. **角度归一化**：`atan2(sin(θ), cos(θ))` 保持 [-π, π]

#### Step 4: 列表重构 (Clean Replacement)

```cpp
// 在 obstaclesCallback 中：
std::vector<EllipseObstacle> merged_obstacles = mergeObstacles(new_obstacles, &debug_info);

{
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_ = merged_obstacles;  // 完全替代，不保留原始
}
```

**关键点：**
- 完全替换 `obstacles_` 成员变量
- 不保留任何原始障碍物
- 向量场计算直接使用 `obstacles_`

---

## 第二部分：智能转向锁定机制

### 锁定机制设计哲学

**问题背景：**
- 原策略：每次实时选择最近障碍物的绕行方向（贪心策略）
- 问题：在虚拟边界附近徘徊时，方向频繁切换，导致抖动

**解决方案：**
- 进入虚拟边界时 **锁定** 障碍物和绕行方向
- 锁定期间保持不变，直到满足解锁条件

---

### 锁定状态结构 (ObstacleLock)

**在 `vector_field_planner.h` 中添加：**

```cpp
struct ObstacleLock {
    int locked_obstacle_id;         // 锁定的障碍物ID（融合后的ID）
    double locked_K;                // 锁定的绕行方向 (1.0顺时针 或 -1.0逆时针)
    std::set<int> locked_merge_ids; // 锁定时该障碍物包含的所有原始ID
    bool is_locked;                 // 是否处于锁定状态
    ros::Time lock_start_time;      // 锁定开始时间
    
    // 锁定的障碍物形状和位置参数（全局坐标）
    double locked_center_x;         // 锁定的障碍物中心X坐标
    double locked_center_y;         // 锁定的障碍物中心Y坐标
    double locked_a;                // 锁定的障碍物长半轴
    double locked_b;                // 锁定的障碍物短半轴
    double locked_theta;            // 锁定的障碍物朝向角
    
    ObstacleLock() : locked_obstacle_id(-1), locked_K(1.0), is_locked(false),
                     locked_center_x(0), locked_center_y(0), 
                     locked_a(0), locked_b(0), locked_theta(0) {}
};

ObstacleLock obstacle_lock_;
std::mutex lock_mutex_;
```

**成员说明：**
- `locked_merge_ids`：记录融合源ID，用于追踪障碍物即使融合状态变化
- 形状参数：锁定时刻的障碍物完整几何信息
- 互斥锁：保证线程安全（回调与控制循环可能并发）

---

### 锁定触发条件 (checkAndLockObstacle)

**触发时机：** 车辆进入任意障碍物的 **虚拟边界** 时

```cpp
void VectorFieldPlanner::checkAndLockObstacle(double x, double y) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    std::lock_guard<std::mutex> obs_guard(obstacle_mutex_);
    
    // 如果已经锁定，不重复锁定
    if (obstacle_lock_.is_locked) {
        return;
    }
    
    double current_yaw = getCurrentYaw();
    
    // 检查是否进入任何障碍物的虚拟边界
    for (const auto& obstacle : obstacles_) {
        double phi_virtual;
        Eigen::Vector2d grad_virtual;
        
        if (enable_virtual_boundary_) {
            // 虚拟边界使用圆形势场
            circleFunctionAndGradient(x, y, obstacle, virtual_reactive_offset_,
                                     phi_virtual, grad_virtual);
        } else {
            continue;  // 虚拟边界未启用，不锁定
        }
        
        // 如果在虚拟边界内（phi < 0）
        if (phi_virtual < 0.0) {
            // 🔒 执行锁定
            obstacle_lock_.locked_obstacle_id = obstacle.id;
            obstacle_lock_.locked_K = computeK(obstacle.center_x, obstacle.center_y, x, y, current_yaw);
            obstacle_lock_.is_locked = true;
            obstacle_lock_.lock_start_time = ros::Time::now();
            
            // 锁定障碍物的形状、大小和位置（全局坐标）
            obstacle_lock_.locked_center_x = obstacle.center_x;
            obstacle_lock_.locked_center_y = obstacle.center_y;
            obstacle_lock_.locked_a = obstacle.a;
            obstacle_lock_.locked_b = obstacle.b;
            obstacle_lock_.locked_theta = obstacle.theta;
            
            // 记录锁定时的融合ID集合
            obstacle_lock_.locked_merge_ids.clear();
            obstacle_lock_.locked_merge_ids.insert(obstacle.id);
            if (obstacle.is_merged) {
                for (int orig_id : obstacle.merged_from_ids) {
                    obstacle_lock_.locked_merge_ids.insert(orig_id);
                }
            }
            
            ROS_INFO("🔒 锁定障碍物 ID:%d, 位置=(%.2f,%.2f), 形状=(a=%.2f,b=%.2f,θ=%.2f°), K=%.1f, 融合来源数:%zu", 
                     obstacle.id, 
                     obstacle_lock_.locked_center_x, obstacle_lock_.locked_center_y,
                     obstacle_lock_.locked_a, obstacle_lock_.locked_b, 
                     obstacle_lock_.locked_theta * 180.0 / M_PI,
                     obstacle_lock_.locked_K,
                     obstacle_lock_.locked_merge_ids.size());
            break;  // 只锁定第一个满足条件的
        }
    }
}
```

**虚拟边界定义：**
```cpp
void VectorFieldPlanner::circleFunctionAndGradient(
    double x, double y, const EllipseObstacle& ellipse,
    double offset, double& phi_val, Eigen::Vector2d& grad) {
    
    double cx = ellipse.center_x;
    double cy = ellipse.center_y;
    double R = ellipse.a + offset;  // 使用长半轴作为圆半径
    
    double dx = x - cx;
    double dy = y - cy;
    double dist_sq = dx * dx + dy * dy;
    
    // 归一化圆形势场
    phi_val = dist_sq / (R * R) - 1.0;
    
    // 梯度
    grad << 2.0 * dx / (R * R), 2.0 * dy / (R * R);
}
```

---

### 锁定期间的障碍物融合处理

**关键策略：锁定障碍物不参与融合，保持锁定时的形状**

```cpp
// 在 obstaclesCallback 中：
if (has_locked_obstacle) {
    // 🔒 锁定状态处理策略：
    // 1. 从新障碍物中移除所有锁定的源ID
    // 2. 检查新障碍物是否与锁定区域重叠
    // 3. 保持锁定障碍物的原始形状不变
    
    std::vector<EllipseObstacle> unlocked_obstacles;
    int removed_by_id = 0;
    int removed_by_overlap = 0;
    std::set<int> newly_absorbed_ids;
    
    for (const auto& obs : new_obstacles) {
        // 检查1：是否是锁定的源ID
        if (locked_merge_ids_copy.count(obs.id) > 0) {
            removed_by_id++;
            continue;
        }
        
        // 检查2：是否与锁定区域重叠
        cv::RotatedRect locked_reactive_obb = buildReactiveOBB(locked_obstacle_copy);
        cv::RotatedRect obs_reactive_obb = buildReactiveOBB(obs);
        
        std::vector<cv::Point2f> intersection_points;
        int intersection_result = cv::rotatedRectangleIntersection(
            locked_reactive_obb, obs_reactive_obb, intersection_points);
        
        if (intersection_result == cv::INTERSECT_PARTIAL || 
            intersection_result == cv::INTERSECT_FULL) {
            removed_by_overlap++;
            newly_absorbed_ids.insert(obs.id);
            ROS_INFO("🔒 锁定期间检测到重叠障碍物 ID:%d，将其吸收", obs.id);
            continue;
        }
        
        // 无关障碍物
        unlocked_obstacles.push_back(obs);
    }
    
    // 更新锁定的融合ID集合
    if (!newly_absorbed_ids.empty()) {
        std::lock_guard<std::mutex> lock_guard(lock_mutex_);
        for (int id : newly_absorbed_ids) {
            obstacle_lock_.locked_merge_ids.insert(id);
            locked_obstacle_copy.merged_from_ids.push_back(id);
        }
    }
    
    // 只对未锁定的障碍物进行融合
    std::vector<EllipseObstacle> unlocked_merged = mergeObstacles(unlocked_obstacles, &debug_info);
    
    // 将锁定的障碍物添加到结果中
    merged_obstacles = unlocked_merged;
    merged_obstacles.push_back(locked_obstacle_copy);
}
```

**关键点：**
- 锁定的障碍物使用 **锁定时的参数**，不更新
- 新出现的重叠障碍物被 **吸收**，ID 加入 `locked_merge_ids`
- 即使锁定的某些源ID消失，锁定的形状也保持不变

---

### 锁定期间的向量场计算

**在向量场计算中使用锁定的参数：**

```cpp
// 在 compositeVectorField 中：
for (const auto& obstacle : obstacles_) {
    // 🔒 判断是否应该使用锁定的障碍物参数
    EllipseObstacle effective_obstacle = obstacle;
    bool use_locked_params = false;
    
    {
        std::lock_guard<std::mutex> lock_guard(lock_mutex_);
        if (obstacle_lock_.is_locked) {
            // 检查当前障碍物是否是锁定的障碍物
            if (obstacle.id == obstacle_lock_.locked_obstacle_id ||
                obstacle_lock_.locked_merge_ids.count(obstacle.id) > 0) {
                // 使用锁定的参数
                effective_obstacle.center_x = obstacle_lock_.locked_center_x;
                effective_obstacle.center_y = obstacle_lock_.locked_center_y;
                effective_obstacle.a = obstacle_lock_.locked_a;
                effective_obstacle.b = obstacle_lock_.locked_b;
                effective_obstacle.theta = obstacle_lock_.locked_theta;
                use_locked_params = true;
            }
        }
    }
    
    // 使用 effective_obstacle 进行所有距离场和梯度计算
    double phi_repulsive, phi_reactive, phi_virtual;
    Eigen::Vector2d grad_phi_repulsive, grad_phi_reactive, grad_phi_virtual;
    
    ellipseFunctionAndGradient(x, y, effective_obstacle, repulsive_offset_,
                              phi_repulsive, grad_phi_repulsive);
    // ... 后续计算
}
```

**getLockedOrComputeK 函数：**

```cpp
double VectorFieldPlanner::getLockedOrComputeK(const EllipseObstacle& obstacle, 
                                               double x, double y, double current_yaw) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    
    if (!obstacle_lock_.is_locked) {
        return computeK(obstacle.center_x, obstacle.center_y, x, y, current_yaw);
    }
    
    // 检查是否是锁定的障碍物
    if (obstacle.id == obstacle_lock_.locked_obstacle_id ||
        obstacle_lock_.locked_merge_ids.count(obstacle.id) > 0) {
        return obstacle_lock_.locked_K;  // 返回锁定的 K 值
    }
    
    // 不是锁定的障碍物，计算新的 K
    return computeK(obstacle.center_x, obstacle.center_y, x, y, current_yaw);
}
```

---

### 解锁条件 (checkAndUnlockObstacle)

**两个解锁条件（满足任一即解锁）：**

#### 条件1：所有锁定的源ID都从输入中消失

```cpp
// 在 obstaclesCallback 处理新障碍物之前检查
if (has_locked_obstacle) {
    bool any_locked_id_exists = false;
    
    for (const auto& obs : new_obstacles) {
        if (obstacle_lock_.locked_merge_ids.count(obs.id) > 0) {
            any_locked_id_exists = true;
            break;
        }
    }
    
    if (!any_locked_id_exists) {
        ROS_INFO("🔓 解锁障碍物 (所有源ID已消失), 锁定时长:%.2fs", 
                 (ros::Time::now() - obstacle_lock_.lock_start_time).toSec());
        
        obstacle_lock_.is_locked = false;
        obstacle_lock_.locked_obstacle_id = -1;
        obstacle_lock_.locked_merge_ids.clear();
        has_locked_obstacle = false;
    }
}
```

#### 条件2：检测到新的障碍物与锁定区域重叠

```cpp
// 在上述检查中同时进行
if (has_locked_obstacle) {
    bool any_locked_id_exists = false;
    bool has_new_overlapping_obstacle = false;
    
    EllipseObstacle temp_locked;
    temp_locked.center_x = obstacle_lock_.locked_center_x;
    temp_locked.center_y = obstacle_lock_.locked_center_y;
    temp_locked.a = obstacle_lock_.locked_a;
    temp_locked.b = obstacle_lock_.locked_b;
    temp_locked.theta = obstacle_lock_.locked_theta;
    
    cv::RotatedRect locked_reactive_obb = buildReactiveOBB(temp_locked);
    
    for (const auto& obs : new_obstacles) {
        if (obstacle_lock_.locked_merge_ids.count(obs.id) > 0) {
            any_locked_id_exists = true;
        } else {
            // 检查新障碍物是否与锁定区域重叠
            cv::RotatedRect obs_reactive_obb = buildReactiveOBB(obs);
            std::vector<cv::Point2f> intersection_points;
            int result = cv::rotatedRectangleIntersection(
                locked_reactive_obb, obs_reactive_obb, intersection_points);
            
            if (result == cv::INTERSECT_PARTIAL || result == cv::INTERSECT_FULL) {
                has_new_overlapping_obstacle = true;
                ROS_INFO("🔓 检测到新障碍物 ID:%d 与锁定区域重叠，将解锁并重新融合", obs.id);
                break;
            }
        }
    }
    
    // 解锁
    if (!any_locked_id_exists || has_new_overlapping_obstacle) {
        obstacle_lock_.is_locked = false;
        obstacle_lock_.locked_obstacle_id = -1;
        obstacle_lock_.locked_merge_ids.clear();
        has_locked_obstacle = false;
    }
}
```

**解锁逻辑总结：**
- **条件1**：障碍物消失（所有源ID不在输入中）
- **条件2**：新障碍物出现且与锁定区域重叠（需要重新融合计算新边界）
- 一旦解锁，下一次进入虚拟边界会重新锁定

---

## 第三部分：OpenCV 调试窗口

### 窗口设计原则

**目标：与 obs_2d 节点的显示风格和坐标系完全一致**

#### 坐标系定义

**obs_2d 的坐标系：**
- **全局坐标** → **车辆坐标系** → **镜像** → **像素坐标**
- X轴和Y轴都取负（双重镜像）
- 车辆始终在原点，朝向固定为 Y 轴正方向

**转换步骤：**

```cpp
// 1. 全局坐标转车辆坐标系
auto globalToVehicleFrame = [&](double global_x, double global_y, 
                                 double& vehicle_x, double& vehicle_y) {
    // 相对于车辆的偏移
    double dx = global_x - car_x;
    double dy = global_y - car_y;
    
    // 旋转到车辆坐标系
    double cos_yaw = std::cos(-car_yaw);
    double sin_yaw = std::sin(-car_yaw);
    double rotated_x = cos_yaw * dx - sin_yaw * dy;
    double rotated_y = sin_yaw * dx + cos_yaw * dy;
    
    // 镜像（与 obs_2d 一致）
    vehicle_x = -rotated_x;
    vehicle_y = -rotated_y;
};

// 2. 车辆坐标系转像素坐标
auto vehicleFrameToPixel = [&](double vehicle_x, double vehicle_y) -> cv::Point {
    int px = origin_x + static_cast<int>(vehicle_x * scale_x);
    int py = origin_y - static_cast<int>(vehicle_y * scale_y);  // Y轴翻转
    return cv::Point(px, py);
};

// 3. 组合：全局坐标直接转像素
auto globalToPixel = [&](double global_x, double global_y) -> cv::Point {
    double vehicle_x, vehicle_y;
    globalToVehicleFrame(global_x, global_y, vehicle_x, vehicle_y);
    return vehicleFrameToPixel(vehicle_x, vehicle_y);
};
```

**角度转换：**

```cpp
// 障碍物角度也需要镜像
double angle_relative = obs.theta - car_yaw;  // 相对角度
double mirrored_angle = -angle_relative;      // 镜像
double angle_deg = mirrored_angle * 180.0 / M_PI;  // 转度数
```

---

### 窗口布局参数

```cpp
const int window_size = 600;           // 窗口大小 600x600
const double scale_y = window_size / 16.0;  // Y轴范围16米 (-2 到 12)
const double scale_x = scale_y;        // 使用统一比例

const int origin_x = window_size / 2;  // X轴原点在中间
const int origin_y = window_size - static_cast<int>(4.0 * scale_y);  // Y=-2 位置
```

**坐标范围：**
- X: -5m 到 +5m（左右各5米）
- Y: -2m 到 +12m（后2米，前12米）

---

### 绘制椭圆边界函数

```cpp
auto drawEllipseBoundary = [&](const EllipseObstacle& obs, double offset, 
                                 const cv::Scalar& color, int thickness) {
    // 椭圆参数
    double a = (obs.a + offset) * 2.0;  // 直径
    double b = (obs.b + offset) * 2.0;
    
    // 障碍物中心：全局坐标转像素
    cv::Point center = globalToPixel(obs.center_x, obs.center_y);
    
    // 椭圆轴长度（像素）
    cv::Size axes(static_cast<int>(a * scale_x / 2.0), 
                 static_cast<int>(b * scale_y / 2.0));
    
    // 角度：镜像处理
    double angle_relative = obs.theta - car_yaw;
    double mirrored_angle = -angle_relative;
    double angle_deg = mirrored_angle * 180.0 / M_PI;
    
    cv::ellipse(canvas, center, axes, angle_deg, 0, 360, color, thickness);
};
```

---

### 配色方案

**正常障碍物：**
- 物理边界（Physical）：黑色 `cv::Scalar(0, 0, 0)`，粗线 2px
- 排斥边界（Repulsive）：红色 `cv::Scalar(0, 0, 255)`，粗线 2px
- 反应边界（Reactive）：绿色 `cv::Scalar(0, 255, 0)`，粗线 2px
- 虚拟边界（Virtual）：淡蓝色 `cv::Scalar(255, 200, 150)`，细线 1px

**锁定障碍物（Locked）：**
- 物理边界：洋红色 `cv::Scalar(255, 0, 255)`，粗线 4px
- 排斥边界：深紫色 `cv::Scalar(200, 0, 200)`，粗线 3px
- 反应边界：洋红色 `cv::Scalar(255, 0, 255)`，粗线 3px
- 虚拟边界：亮紫色 `cv::Scalar(255, 128, 255)`，粗线 3px

**其他元素：**
- 坐标轴：黑色 2px
- 网格线：灰色 `cv::Scalar(200, 200, 200)` 1px
- 车辆矩形：红色 `cv::Scalar(0, 0, 255)` 3px
- 车辆中心点：红色实心圆

---

### 锁定状态显示

```cpp
// 获取锁定状态
bool is_locked = false;
int locked_id = -1;
std::set<int> locked_merge_ids;
EllipseObstacle locked_obstacle;

{
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    is_locked = obstacle_lock_.is_locked;
    if (is_locked) {
        locked_id = obstacle_lock_.locked_obstacle_id;
        locked_merge_ids = obstacle_lock_.locked_merge_ids;
        
        // 创建锁定的障碍物对象用于可视化
        locked_obstacle.id = locked_id;
        locked_obstacle.center_x = obstacle_lock_.locked_center_x;
        locked_obstacle.center_y = obstacle_lock_.locked_center_y;
        locked_obstacle.a = obstacle_lock_.locked_a;
        locked_obstacle.b = obstacle_lock_.locked_b;
        locked_obstacle.theta = obstacle_lock_.locked_theta;
        locked_obstacle.is_merged = !locked_merge_ids.empty();
    }
}

// 绘制障碍物时检查是否被锁定
for (const auto& obs : final_obstacles) {
    bool is_this_locked = false;
    EllipseObstacle display_obs = obs;
    
    if (is_locked && (obs.id == locked_id || locked_merge_ids.count(obs.id) > 0)) {
        is_this_locked = true;
        display_obs = locked_obstacle;  // 使用锁定的参数
    }
    
    // 根据 is_this_locked 选择颜色和线宽
    cv::Scalar physical_color = is_this_locked ? 
        cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 0);
    int physical_thickness = is_this_locked ? 4 : 2;
    
    drawEllipseBoundary(display_obs, 0.0, physical_color, physical_thickness);
    // ... 其他边界
    
    // ID标签
    cv::Point center = globalToPixel(display_obs.center_x, display_obs.center_y);
    std::string id_text = display_obs.is_merged ? 
        ("M" + std::to_string(display_obs.id)) : std::to_string(display_obs.id);
    if (is_this_locked) {
        id_text = "🔒" + id_text;  // 添加锁定标记
    }
    cv::Scalar text_color = is_this_locked ? 
        cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 0);
    cv::putText(canvas, id_text, center, cv::FONT_HERSHEY_SIMPLEX, 
               0.5, text_color, 2);
}

// 显示锁定状态信息
if (is_locked) {
    std::string lock_info = "LOCKED: Obs#" + std::to_string(locked_id);
    cv::putText(canvas, lock_info, cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    info_y += 25;
}
```

---

### 完整的可视化函数框架

```cpp
void VectorFieldPlanner::visualizeMergingProcess(const MergeDebugInfo& debug_info) {
    // 1. 初始化窗口
    if (!merge_viz_initialized_) {
        cv::namedWindow(merge_window_name_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(merge_window_name_, window_size, window_size);
        merge_viz_initialized_ = true;
    }
    
    // 2. 创建白色背景
    cv::Mat canvas = cv::Mat(window_size, window_size, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // 3. 获取车辆位置和朝向
    double car_x = 0.0, car_y = 0.0, car_yaw = 0.0;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (has_pose_) {
            car_x = current_pose_.position.x;
            car_y = current_pose_.position.y;
            car_yaw = extractYawFromQuaternion(current_pose_.orientation);
        }
    }
    
    // 4. 绘制坐标系和网格
    drawCoordinateSystem(canvas);
    drawGrid(canvas);
    
    // 5. 绘制车辆
    drawVehicle(canvas);
    
    // 6. 获取锁定状态
    bool is_locked = false;
    // ... (如上)
    
    // 7. 绘制所有障碍物
    std::vector<EllipseObstacle> final_obstacles;
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        final_obstacles = obstacles_;
    }
    
    for (const auto& obs : final_obstacles) {
        // 检查是否锁定
        // 选择颜色和线宽
        // 绘制四层边界
        // 绘制ID标签
    }
    
    // 8. 显示UI信息
    displayInfo(canvas, final_obstacles, is_locked, locked_id);
    
    // 9. 显示窗口
    cv::imshow(merge_window_name_, canvas);
}
```

---

## 第四部分：关键实现细节

### 头文件修改清单

**在 `vector_field_planner.h` 中添加：**

```cpp
#include <opencv2/opencv.hpp>
#include <set>

struct ObstacleLock {
    // ... (完整定义见上文)
};

// 成员变量
ObstacleLock obstacle_lock_;
std::mutex lock_mutex_;

// 可视化相关
bool merge_viz_initialized_;
std::string merge_window_name_;

// 函数声明
std::vector<EllipseObstacle> mergeObstacles(
    const std::vector<EllipseObstacle>& obstacles,
    MergeDebugInfo* debug_info);

cv::RotatedRect buildReactiveOBB(const EllipseObstacle& obstacle);

void checkAndLockObstacle(double x, double y);
void checkAndUnlockObstacle(double x, double y);
double getLockedOrComputeK(const EllipseObstacle& obstacle, 
                           double x, double y, double current_yaw);

void visualizeMergingProcess(const MergeDebugInfo& debug_info);

void circleFunctionAndGradient(double x, double y, 
                               const EllipseObstacle& ellipse,
                               double offset, 
                               double& phi_val, 
                               Eigen::Vector2d& grad);
```

---

### 源文件包含清单

**在 `vector_field_planner.cpp` 开头添加：**

```cpp
#include "bringup/vector_field_planner.h"
#include <opencv2/opencv.hpp>
#include <set>
#include <algorithm>
#include <numeric>
```

---

### 障碍物回调函数完整流程

```cpp
void VectorFieldPlanner::obstaclesCallback(const std_msgs::String::ConstPtr& msg) {
    std::vector<EllipseObstacle> new_obstacles;
    
    // 1. 解析输入数据（正则表达式提取ID、中心、大小、角度）
    parseObstacleMessage(msg->data, new_obstacles);
    
    // 2. 坐标转换（velodyne → map）
    transformToMapFrame(new_obstacles);
    
    // 3. 检查锁定状态并决定是否解锁
    bool has_locked_obstacle = false;
    EllipseObstacle locked_obstacle_copy;
    std::set<int> locked_merge_ids_copy;
    
    {
        std::lock_guard<std::mutex> lock_guard(lock_mutex_);
        has_locked_obstacle = obstacle_lock_.is_locked;
        
        if (has_locked_obstacle) {
            // 检查解锁条件1和2
            // ... (详见解锁条件章节)
        }
        
        if (has_locked_obstacle) {
            // 保存锁定的障碍物信息
            // ...
        }
    }
    
    // 4. 障碍物融合
    MergeDebugInfo debug_info;
    std::vector<EllipseObstacle> merged_obstacles;
    
    if (has_locked_obstacle) {
        // 锁定状态：分离处理
        // ... (详见锁定期间融合章节)
    } else {
        // 正常状态：全部融合
        merged_obstacles = mergeObstacles(new_obstacles, &debug_info);
    }
    
    // 5. 更新全局障碍物列表
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        obstacles_ = merged_obstacles;
    }
    
    // 6. 可视化
    visualizeMergingProcess(debug_info);
}
```

---

### 向量场计算函数集成点

**在 `compositeVectorField` 函数中：**

```cpp
Eigen::Vector2d VectorFieldPlanner::compositeVectorField(double x, double y) {
    // 1. 在函数开始检查锁定状态
    checkAndLockObstacle(x, y);
    checkAndUnlockObstacle(x, y);
    
    // 2. 计算路径跟踪向量场
    Eigen::Vector2d chi_P = pathTrackingVectorField(x, y);
    
    // 3. 遍历障碍物计算复合向量场
    for (const auto& obstacle : obstacles_) {
        // 🔒 使用锁定的参数（如果适用）
        EllipseObstacle effective_obstacle = getEffectiveObstacle(obstacle);
        
        // 计算距离场和梯度
        // ...
        
        // 🔒 使用锁定的 K 值（如果适用）
        double K = getLockedOrComputeK(obstacle, x, y, current_yaw);
        
        // 计算反应向量
        // ...
    }
    
    // 4. 返回合成向量
    return chi_c_normalized;
}
```

---

### 主循环集成

**在 `run()` 函数中：**

```cpp
void VectorFieldPlanner::run() {
    ros::Rate rate(control_rate_);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // ... 获取位置和朝向
        
        // 获取控制向量（内部会调用 compositeVectorField）
        Eigen::Vector2d chi_c = getVectorAtPosition(front_x, front_y);
        
        // 转换为控制命令
        geometry_msgs::Twist cmd = vectorToAckermann(chi_c, current_yaw);
        
        // 发布控制命令
        cmd_vel_pub_.publish(cmd);
        
        // 可视化（障碍物边界和当前向量）
        visualizeObstacleBoundaries();
        visualizeCurrentVector(front_x, front_y, chi_c, current_yaw);
        
        // 🔒 处理OpenCV窗口事件（必须在主循环中调用）
        cv::waitKey(1);
        
        rate.sleep();
    }
}
```

---

## 第五部分：测试验证清单

### 功能测试项

#### 1. 障碍物融合测试
- [ ] 两个障碍物反应边界重叠 → 融合为一个
- [ ] 三个障碍物传递重叠（A⊕B, B⊕C） → 融合为一个
- [ ] 两个簇独立 → 生成两个融合障碍物
- [ ] 单个障碍物 → 保持原样
- [ ] 融合后的尺寸合理（不小于原始障碍物）

#### 2. 锁定机制测试
- [ ] 进入虚拟边界 → 触发锁定（日志：🔒 锁定障碍物）
- [ ] 锁定后绕行方向保持不变
- [ ] 锁定后障碍物形状保持不变（即使实时数据变化）
- [ ] 锁定期间新出现的重叠障碍物被吸收
- [ ] 所有源ID消失 → 解锁（日志：🔓 解锁障碍物）
- [ ] 新障碍物与锁定区域重叠 → 解锁并重新融合

#### 3. 可视化测试
- [ ] 坐标系与 obs_2d 一致（车辆固定在原点）
- [ ] 障碍物位置与 obs_2d 一致（偏差 < 0.3m）
- [ ] 锁定的障碍物显示为洋红色
- [ ] 锁定的障碍物形状固定（不随实时数据变化）
- [ ] 窗口显示 "LOCKED: Obs#X"
- [ ] ID 标签前显示 🔒 符号

#### 4. 边界情况测试
- [ ] 障碍物快速移动 → 融合结果稳定
- [ ] 传感器丢失部分障碍物 → 锁定的障碍物保持
- [ ] 车辆在虚拟边界徘徊 → 方向不频繁切换
- [ ] 多个障碍物同时进入虚拟边界 → 只锁定第一个

---

### 性能验证

**期望指标：**
- 融合算法耗时：< 5ms（10个障碍物）
- 向量场计算耗时：< 2ms（单点查询）
- 控制频率：≥ 50Hz
- OpenCV 窗口刷新率：≥ 10Hz
- 内存占用：增加 < 50MB

---

## 第六部分：常见问题与解决方案

### Q1: 融合后的障碍物过大

**原因：**
- `reactive_offset_` 设置过大
- 凸包计算包含了过多无关点

**解决：**
```cpp
// 调整 reactive_offset_
reactive_offset_ = 0.3;  // 推荐范围 0.2-0.5

// 凸包计算后检查尺寸
if (new_a > original_max_a * 2.0) {
    ROS_WARN("融合后尺寸异常，使用保守估计");
    new_a = original_max_a * 1.5;
}
```

### Q2: 坐标系不一致

**原因：**
- 忘记镜像 X 和 Y
- 角度转换错误

**解决：**
```cpp
// 确保双重镜像
vehicle_x = -rotated_x;
vehicle_y = -rotated_y;

// 角度也要镜像
mirrored_angle = -angle_relative;
```

### Q3: 锁定后方向频繁切换

**原因：**
- 解锁条件过于宽松
- 没有正确检查新障碍物重叠

**解决：**
```cpp
// 确保解锁条件严格
if (!any_locked_id_exists || has_new_overlapping_obstacle) {
    // 两个条件是 OR 关系
}
```

### Q4: 编译错误缺少 OpenCV 函数

**解决：**
```cmake
# CMakeLists.txt 中添加
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(your_node ${OpenCV_LIBS})
```

### Q5: 锁定的障碍物在画面上漂移

**原因：**
- 使用了实时数据而不是锁定的参数

**解决：**
```cpp
// 确保使用锁定的参数
if (is_this_locked) {
    display_obs = locked_obstacle;  // 不是 obs
}
```

---

## 第七部分：参数调优指南

### 关键参数表

| 参数名 | 推荐范围 | 默认值 | 说明 |
|--------|----------|--------|------|
| `reactive_offset_` | 0.2-0.5m | 0.3m | 反应边界膨胀量，影响融合触发距离 |
| `repulsive_offset_` | 0.1-0.3m | 0.15m | 排斥边界膨胀量 |
| `virtual_reactive_offset_` | 1.0-3.0m | 1.5m | 虚拟边界半径，影响锁定触发距离 |
| `k_r_` | 0.5-2.0 | 1.0 | 反应向量场增益 |
| `lookahead_distance_` | 0.5-2.0m | 1.0m | 前瞻距离 |

### 调参策略

**场景1：密集障碍物环境**
```cpp
reactive_offset_ = 0.4;  // 增大，提前融合
virtual_reactive_offset_ = 2.0;  // 增大，提前锁定
```

**场景2：稀疏障碍物环境**
```cpp
reactive_offset_ = 0.25;  // 减小，避免过度融合
virtual_reactive_offset_ = 1.2;  // 减小，延迟锁定
```

**场景3：高速场景**
```cpp
lookahead_distance_ = 1.5;  // 增大前瞻
virtual_reactive_offset_ = 2.5;  // 增大锁定范围
```

---

## 代码交付检查清单

### 文件修改清单

- [x] `vector_field_planner.h`
  - [x] 添加 `#include <opencv2/opencv.hpp>`
  - [x] 添加 `#include <set>`
  - [x] 定义 `ObstacleLock` 结构
  - [x] 声明融合函数
  - [x] 声明锁定相关函数
  - [x] 声明可视化函数
  - [x] 添加成员变量

- [x] `vector_field_planner.cpp`
  - [x] 实现 `buildReactiveOBB`
  - [x] 实现 `mergeObstacles`（包含 Union-Find）
  - [x] 实现 `checkAndLockObstacle`
  - [x] 实现 `checkAndUnlockObstacle`
  - [x] 实现 `getLockedOrComputeK`
  - [x] 实现 `circleFunctionAndGradient`
  - [x] 修改 `obstaclesCallback`（添加解锁检查和锁定处理）
  - [x] 修改 `compositeVectorField`（添加锁定检查和参数使用）
  - [x] 实现 `visualizeMergingProcess`
  - [x] 修改 `run`（添加 `cv::waitKey(1)`）

- [x] `vector_field_planner_viz.cpp`（如果分离可视化代码）
  - [x] 实现完整的 OpenCV 可视化

### 编译验证

```bash
cd /path/to/workspace
catkin_make -DVTK_DIR=/usr/local/lib/cmake/vtk-9.4 -DPCL_SHARED_LIBS=ON

# 检查输出
# [100%] Built target vector_field_planner_cpp
```

### 运行验证

```bash
roslaunch bringup circular_path_navigation_cpp.launch

# 观察日志
# [ INFO] 🔒 锁定障碍物 ID:X, K=1.0, ...
# [ INFO] 🔓 解锁障碍物 ID:X (锁定时长:X.XXs)

# 观察 OpenCV 窗口
# - 锁定的障碍物显示为洋红色
# - 窗口显示 "LOCKED: Obs#X"
```

---

## 总结

本指南涵盖了：
1. **障碍物融合**：基于反应边界 OBB 的完全替代策略
2. **智能锁定**：进入虚拟边界锁定，形状和方向固定
3. **OpenCV 可视化**：坐标系与 obs_2d 一致，锁定状态清晰显示
4. **所有关键细节**：Union-Find、凸包、MinAreaRect、坐标镜像、角度转换

**关键成功因素：**
- 严格的完全替代原则
- 准确的坐标转换（双重镜像）
- 正确的锁定和解锁逻辑
- 线程安全的状态管理

**遵循本指南，您可以从零开始实现一个完整的、稳定的、可视化良好的障碍物融合与锁定系统。**
