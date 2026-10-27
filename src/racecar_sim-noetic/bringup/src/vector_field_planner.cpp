#include "bringup/vector_field_planner.h"
#include "bringup/ControlDebug.h"
#include <regex>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <set>

namespace vector_field_planner {

VectorFieldPlanner::VectorFieldPlanner()
    : private_nh_("~"), has_pose_(false), has_velocity_(false) {
    
    // ========== 加载参数 ==========
    private_nh_.param("k_p", k_p_, 1.0);
    private_nh_.param("k_r", k_r_, 1.5);
    private_nh_.param("l1", l1_, 0.3);
    private_nh_.param("l2", l2_, 0.3);
    private_nh_.param("bump_transition_width", bump_transition_width_, 10.0);  // 保留用于兼容
    private_nh_.param("c", c_, 0.0);  // 论文中的常数c，默认为0
    private_nh_.param("repulsive_offset", repulsive_offset_, 0.6);
    private_nh_.param("reactive_offset", reactive_offset_, 1.0);
    
    private_nh_.param("max_speed", max_vector_norm_, 2.0);
    private_nh_.param("control_rate", control_rate_, 10.0);
    private_nh_.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh_.param("wheelbase", wheelbase_, 0.335);
    private_nh_.param("max_steering_angle", max_steering_angle_, 0.6);
    private_nh_.param("steering_gain", steering_gain_, 10.0);
    
    // 计算最小转弯半径 R = L / tan(delta_max)
    if (std::abs(max_steering_angle_) > epsilon_) {
        min_turning_radius_ = wheelbase_ / std::tan(max_steering_angle_);
    } else {
        min_turning_radius_ = 100.0;  // 极大值，表示几乎无法转弯
    }
    ROS_INFO("最小转弯半径: %.3f m (轴距=%.3fm, 最大转角=%.2f°)", 
             min_turning_radius_, wheelbase_, max_steering_angle_ * 180.0 / M_PI);
    
    // 前瞻距离参数
    private_nh_.param("lookahead_distance", lookahead_distance_, 0.2);
    
    // 路径参数
    private_nh_.param<std::string>("path_type", path_type_, "circle");
    
    // 圆形路径参数
    private_nh_.param("circle_center_x", circle_center_x_, 0.0);
    private_nh_.param("circle_center_y", circle_center_y_, 0.0);
    private_nh_.param("circle_radius", circle_radius_, 5.0);
    private_nh_.param<std::string>("circle_direction", circle_direction_, "ccw");
    
    // 线段路径参数
    private_nh_.param("line_start_x", line_start_x_, -5.0);
    private_nh_.param("line_start_y", line_start_y_, 0.0);
    private_nh_.param("line_end_x", line_end_x_, 5.0);
    private_nh_.param("line_end_y", line_end_y_, 0.0);
    private_nh_.param<std::string>("line_direction", line_direction_, "forward");
    
    // 8字形路径参数
    private_nh_.param("figure8_scale", figure8_scale_, 3.0);
    private_nh_.param<std::string>("figure8_direction", figure8_direction_, "ccw");
    
    // 向量场参数
    private_nh_.param("field_center_x", field_center_x_, 0.0);
    private_nh_.param("field_center_y", field_center_y_, 0.0);
    private_nh_.param("field_range", field_range_, 10.0);
    private_nh_.param("field_resolution", field_resolution_, 0.3);
    private_nh_.param("viz_frequency", viz_frequency_, 10.0);
    
    // 局部路径参数
    private_nh_.param("local_path_length", local_path_length_, 3.0);
    private_nh_.param("local_path_step", local_path_step_, 0.1);
    
    // 障碍物管理参数
    private_nh_.param("obstacle_association_threshold", obstacle_association_threshold_, 1.0);
    // private_nh_.param("obstacle_duration_time", obstacle_duration_time_, 0.5);
    
    // 数值稳定性参数
    epsilon_ = 1e-8;
    min_vector_norm_ = 1e-4;
    
    // 旋转矩阵 E = [[0, -1], [1, 0]]
    E_ << 0.0, -1.0,
          1.0,  0.0;
    
    field_update_interval_ = 1.0 / viz_frequency_;
    last_viz_time_ = ros::Time(0);
    last_field_update_time_ = ros::Time(0);
    
    // ========== 初始化订阅器 ==========
    odom_sub_ = nh_.subscribe("/lio_sam/mapping/odometry", 1, 
                              &VectorFieldPlanner::odomCallback, this);
    imu_odom_sub_ = nh_.subscribe("/odometry/imu", 1, 
                                  &VectorFieldPlanner::imuOdomCallback, this);
    obstacles_sub_ = nh_.subscribe("/obs_fitting/ellipse_equations", 1, 
                                   &VectorFieldPlanner::obstaclesCallback, this);
    
    // ========== 初始化发布器 ==========
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    local_plan_pub_ = nh_.advertise<nav_msgs::Path>("/vector_field_planner/local_plan", 1);
    vector_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/vector_field", 1);
    path_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/path_field", 1);
    reactive_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/reactive_field", 1);
    obstacle_boundaries_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/obstacle_boundaries", 1);
    yaw_error_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/yaw_error", 1);
    current_vector_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/vector_field_planner/current_vector", 1);
    circle_path_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/vector_field_planner/circle_path", 1, true);
    field_compute_time_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/field_compute_time", 1);
    control_query_time_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/control_query_time", 1);
    viz_time_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/viz_time", 1);
    control_debug_pub_ = nh_.advertise<bringup::ControlDebug>(
        "/vector_field_planner/control_debug", 1);
    step_times_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
        "/vector_field_planner/step_times", 1);
    bypass_direction_pub_ = nh_.advertise<std_msgs::String>(
        "/vector_field_planner/bypass_direction", 1);  // 绕行方向信息
    
    // 发布路径可视化
    if (path_type_ == "circle") {
        visualizeCirclePath();
    } else if (path_type_ == "line") {
        visualizeLinePath();
    } else if (path_type_ == "figure8") {
        visualizeFigure8Path();
    }
    
    ROS_INFO("向量场规划器初始化完成 (基于极限圆的动态方向决策)");
}

// ========== 回调函数 ==========
void VectorFieldPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = msg->pose.pose;
    has_pose_ = true;
}

void VectorFieldPlanner::imuOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_velocity_ = msg->twist.twist;
    has_velocity_ = true;
}

void VectorFieldPlanner::obstaclesCallback(const std_msgs::String::ConstPtr& msg) {
    std::vector<EllipseObstacle> new_obstacles;
    
    try {
        std::string data = msg->data;
        std::istringstream iss(data);
        std::string line;
        
        // ID正则匹配: "椭圆 ID:123"
        std::regex id_regex(R"(ID:(-?\d+))");
        std::regex center_regex(R"(中心:\(([^,]+),([^)]+)\))");
        std::regex size_regex(R"(大小:([^x]+)x([^\s]+))");
        std::regex angle_regex(R"(角度:([^°]+)°)");
        
        while (std::getline(iss, line)) {
            if (line.find("中心:") == std::string::npos) continue;
            
            std::smatch id_match, center_match, size_match, angle_match;
            
            // 提取ID
            int stable_id = -1;
            if (std::regex_search(line, id_match, id_regex)) {
                stable_id = std::stoi(id_match[1]);
            }
            
            // 提取中心坐标
            if (!std::regex_search(line, center_match, center_regex)) continue;
            double center_x_velodyne = std::stod(center_match[1]);
            double center_y_velodyne = std::stod(center_match[2]);
            
            // 提取大小
            if (!std::regex_search(line, size_match, size_regex)) continue;
            double width = std::stod(size_match[1]);
            double height = std::stod(size_match[2]);
            
            // 提取角度
            if (!std::regex_search(line, angle_match, angle_regex)) continue;
            double angle_velodyne_deg = std::stod(angle_match[1]);
            double angle_velodyne_rad = angle_velodyne_deg * M_PI / 180.0;
            
            // 坐标转换: velodyne -> map
            try {
                geometry_msgs::PointStamped point_velodyne, point_map;
                point_velodyne.header.frame_id = "velodyne";
                point_velodyne.header.stamp = ros::Time(0);
                point_velodyne.point.x = center_x_velodyne;
                point_velodyne.point.y = center_y_velodyne;
                point_velodyne.point.z = 0.0;
                
                tf_listener_.transformPoint("map", point_velodyne, point_map);
                double center_x = point_map.point.x;
                double center_y = point_map.point.y;
                
                // 获取旋转变换
                tf::StampedTransform transform;
                tf_listener_.lookupTransform("map", "velodyne", ros::Time(0), transform);
                double yaw_map_to_velodyne = tf::getYaw(transform.getRotation());
                
                double angle_map_rad = angle_velodyne_rad + yaw_map_to_velodyne;
                angle_map_rad = atan2(sin(angle_map_rad), cos(angle_map_rad));
                
                // 存入列表
                new_obstacles.emplace_back(stable_id, center_x, center_y, width, height, angle_map_rad);
                
            } catch (tf::TransformException& ex) {
                continue;
            }
        }
        
        // ========== 障碍物融合流水线 ==========
        std::vector<EllipseObstacle> merged_obstacles;
        
        // 检查是否有锁定的融合障碍物
        bool has_locked_fusion = false;
        EllipseObstacle locked_obstacle_copy;
        std::set<int> locked_merge_ids_copy;
        {
            std::lock_guard<std::mutex> lock_guard(lock_mutex_);
            has_locked_fusion = fusion_lock_.is_locked;
            
            // 在处理新障碍物之前，检查是否应该解锁
            if (has_locked_fusion) {
                bool any_locked_id_exists = false;
                bool has_new_overlapping_obstacle = false;
                
                // 创建锁定障碍物的临时副本用于重叠检测
                EllipseObstacle temp_locked;
                temp_locked.center_x = fusion_lock_.locked_center_x;
                temp_locked.center_y = fusion_lock_.locked_center_y;
                temp_locked.a = fusion_lock_.locked_a;
                temp_locked.b = fusion_lock_.locked_b;
                temp_locked.theta = fusion_lock_.locked_theta;
                
                cv::RotatedRect locked_repulsive_obb = buildRepulsiveOBB(temp_locked);
                
                // 遍历新输入的障碍物
                for (const auto& obs : new_obstacles) {
                    // 检查1：是否是锁定的源ID
                    if (fusion_lock_.locked_merge_ids.count(obs.id) > 0) {
                        any_locked_id_exists = true;
                    } else {
                        // 检查2：新障碍物是否与锁定区域重叠
                        cv::RotatedRect obs_repulsive_obb = buildRepulsiveOBB(obs);
                        std::vector<cv::Point2f> intersection_points;
                        int intersection_result = cv::rotatedRectangleIntersection(
                            locked_repulsive_obb, obs_repulsive_obb, intersection_points);
                        
                        if (intersection_result == cv::INTERSECT_PARTIAL || 
                            intersection_result == cv::INTERSECT_FULL) {
                            has_new_overlapping_obstacle = true;
                            ROS_INFO("🔓 检测到新障碍物 ID:%d 与锁定区域重叠，将解锁并重新融合", obs.id);
                            break;
                        }
                    }
                }
                
                // 解锁条件1：所有锁定的源ID都消失了
                if (!any_locked_id_exists) {
                    ROS_INFO("🔓 解锁融合障碍物 ID:%d (所有源ID已消失), 锁定时长:%.2fs", 
                             fusion_lock_.locked_obstacle_id,
                             (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
                    
                    fusion_lock_.is_locked = false;
                    fusion_lock_.locked_obstacle_id = -1;
                    fusion_lock_.locked_merge_ids.clear();
                    has_locked_fusion = false;
                }
                // 解锁条件2：有新的障碍物与锁定区域重叠
                else if (has_new_overlapping_obstacle) {
                    ROS_INFO("🔓 解锁融合障碍物 ID:%d (新障碍物重叠，需重新融合), 锁定时长:%.2fs", 
                             fusion_lock_.locked_obstacle_id,
                             (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
                    
                    fusion_lock_.is_locked = false;
                    fusion_lock_.locked_obstacle_id = -1;
                    fusion_lock_.locked_merge_ids.clear();
                    has_locked_fusion = false;
                }
            }
            
            if (has_locked_fusion) {
                // 保存锁定的障碍物信息
                locked_obstacle_copy.id = fusion_lock_.locked_obstacle_id;
                locked_obstacle_copy.center_x = fusion_lock_.locked_center_x;
                locked_obstacle_copy.center_y = fusion_lock_.locked_center_y;
                locked_obstacle_copy.a = fusion_lock_.locked_a;
                locked_obstacle_copy.b = fusion_lock_.locked_b;
                locked_obstacle_copy.theta = fusion_lock_.locked_theta;
                locked_obstacle_copy.is_merged = !fusion_lock_.locked_merge_ids.empty();
                locked_obstacle_copy.merged_from_ids.clear();
                for (int id : fusion_lock_.locked_merge_ids) {
                    locked_obstacle_copy.merged_from_ids.push_back(id);
                }
                locked_merge_ids_copy = fusion_lock_.locked_merge_ids;
            }
        }
        
        if (has_locked_fusion) {
            // 锁定状态策略：
            // 1. 从新障碍物中移除所有锁定的源ID（防止重新融合）
            // 2. 检查新障碍物是否与锁定障碍物的反应边界重叠，若重叠也移除
            // 3. 即使某些锁定的源ID丢失视野，也保持锁定的大椭圆不变
            // 4. 只对完全不相关的障碍物进行融合
            
            std::vector<EllipseObstacle> unlocked_obstacles;
            int removed_by_id = 0;
            int removed_by_overlap = 0;
            std::set<int> newly_absorbed_ids;
            
            for (const auto& obs : new_obstacles) {
                // 检查1：如果障碍物ID在锁定的融合ID集合中，直接移除
                if (locked_merge_ids_copy.count(obs.id) > 0) {
                    removed_by_id++;
                    continue;
                }
                
                // 检查2：判断新障碍物是否与锁定的障碍物排斥边界重叠
                cv::RotatedRect locked_repulsive_obb = buildRepulsiveOBB(locked_obstacle_copy);
                cv::RotatedRect obs_repulsive_obb = buildRepulsiveOBB(obs);
                
                std::vector<cv::Point2f> intersection_points;
                int intersection_result = cv::rotatedRectangleIntersection(
                    locked_repulsive_obb, obs_repulsive_obb, intersection_points);
                
                if (intersection_result == cv::INTERSECT_PARTIAL || 
                    intersection_result == cv::INTERSECT_FULL) {
                    removed_by_overlap++;
                    newly_absorbed_ids.insert(obs.id);
                    ROS_DEBUG("🔒 锁定期间检测到重叠障碍物 ID:%d 与锁定区域重叠，将其吸收", obs.id);
                    continue;
                }
                
                unlocked_obstacles.push_back(obs);
            }
            
            // 更新锁定的融合ID集合（加入新吸收的ID）
            if (!newly_absorbed_ids.empty()) {
                std::lock_guard<std::mutex> lock_guard(lock_mutex_);
                for (int id : newly_absorbed_ids) {
                    fusion_lock_.locked_merge_ids.insert(id);
                    locked_obstacle_copy.merged_from_ids.push_back(id);
                }
                locked_merge_ids_copy.insert(newly_absorbed_ids.begin(), newly_absorbed_ids.end());
            }
            
            // 只对未锁定的障碍物进行融合
            std::vector<EllipseObstacle> unlocked_merged = mergeObstacles(unlocked_obstacles);
            
            // 将锁定的障碍物添加到结果中
            merged_obstacles = unlocked_merged;
            merged_obstacles.push_back(locked_obstacle_copy);
            
        } else {
            // 正常状态：对所有障碍物进行融合
            merged_obstacles = mergeObstacles(new_obstacles);
        }
        
        // 更新全局障碍物列表（完全替代原则）
        {
            std::lock_guard<std::mutex> lock(obstacle_mutex_);
            obstacles_ = merged_obstacles;
        }
        
    } catch (std::exception& e) {
        ROS_ERROR("处理障碍物消息失败: %s", e.what());
    }
}

// ========== 障碍物融合实现 ==========

/**
 * @brief 构建障碍物的排斥边界OBB
 */
cv::RotatedRect VectorFieldPlanner::buildRepulsiveOBB(const EllipseObstacle& obstacle) {
    // 排斥边界尺寸 = 物理尺寸 + repulsive_offset
    float width = static_cast<float>((obstacle.a + repulsive_offset_) * 2.0);
    float height = static_cast<float>((obstacle.b + repulsive_offset_) * 2.0);
    
    cv::Point2f center(static_cast<float>(obstacle.center_x), 
                      static_cast<float>(obstacle.center_y));
    
    // OpenCV RotatedRect angle is in degrees
    float angle_deg = static_cast<float>(obstacle.theta * 180.0 / M_PI);
    
    return cv::RotatedRect(center, cv::Size2f(width, height), angle_deg);
}

/**
 * @brief 检测两个旋转矩形是否重叠（基于OpenCV）
 */
bool VectorFieldPlanner::checkOBBOverlap(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2) {
    std::vector<cv::Point2f> intersections;
    int result = cv::rotatedRectangleIntersection(rect1, rect2, intersections);
    
    // INTERSECT_NONE = 0, INTERSECT_PARTIAL = 1, INTERSECT_FULL = 2
    return (result == cv::INTERSECT_PARTIAL || result == cv::INTERSECT_FULL);
}

/**
 * @brief 核心融合算法：基于反应边界OBB的完全替代策略
 */
std::vector<EllipseObstacle> VectorFieldPlanner::mergeObstacles(
    const std::vector<EllipseObstacle>& obstacles) {
    
    if (obstacles.empty()) {
        return obstacles;
    }
    
    const size_t n = obstacles.size();
    
    // Step 1: 构建用于重叠检测的 OBB（基于物理尺寸 + merge_safe_offset）
    // 核心原则：只有当两障碍物之间的真实物理间隙 < 2*merge_safe_offset 时才合并，
    // 与力场参数（repulsive_offset / reactive_offset）完全解耦。
    const double merge_safe_offset = 0.6;  // 车辆安全通过所需的单侧余量 (m)
    std::vector<cv::RotatedRect> boundary_obbs;
    boundary_obbs.reserve(n);
    for (const auto& obs : obstacles) {
        // 使用纯物理尺寸膨胀，而非力场尺寸
        cv::Size2f size(
            static_cast<float>((obs.a + merge_safe_offset) * 2.0),
            static_cast<float>((obs.b + merge_safe_offset) * 2.0));
        float angle_deg = static_cast<float>(obs.theta * 180.0 / M_PI);
        boundary_obbs.emplace_back(
            cv::Point2f(static_cast<float>(obs.center_x),
                        static_cast<float>(obs.center_y)),
            size, angle_deg);
    }
    
    // Step 2: 构建邻接矩阵并聚类（Union-Find）
    std::vector<int> parent(n);
    for (size_t i = 0; i < n; ++i) {
        parent[i] = static_cast<int>(i);
    }
    
    // Union-Find: Find with path compression
    std::function<int(int)> find = [&](int x) -> int {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    };
    
    // Union-Find: Union
    auto unite = [&](int x, int y) {
        int px = find(x);
        int py = find(y);
        if (px != py) {
            parent[px] = py;
        }
    };
    
    // 检测重叠并合并
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (checkOBBOverlap(boundary_obbs[i], boundary_obbs[j])) {
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }
    
    // Step 3: 收集聚类结果
    std::map<int, std::vector<int>> cluster_map;
    for (size_t i = 0; i < n; ++i) {
        int root = find(static_cast<int>(i));
        cluster_map[root].push_back(static_cast<int>(i));
    }
    
    // Step 4: 对每个簇进行几何融合
    std::vector<EllipseObstacle> final_obstacles;
    int next_merged_id = 10000;
    
    for (const auto& cluster_pair : cluster_map) {
        const std::vector<int>& cluster_indices = cluster_pair.second;
        
        // 单个障碍物，直接保留
        if (cluster_indices.size() == 1) {
            final_obstacles.push_back(obstacles[cluster_indices[0]]);
            continue;
        }
        
        // 多个障碍物需要融合
        // Step 4.1: 沿每个原始椭圆的纯物理边界均匀采样 12 个点
        // 采样公式（含中心平移和旋转角 theta）：
        //   x_local  = a * cos(t),  y_local  = b * sin(t)
        //   x_global = cx + x_local*cos(θ) - y_local*sin(θ)
        //   y_global = cy + x_local*sin(θ) + y_local*cos(θ)
        const int sample_count = 12;
        std::vector<cv::Point2f> all_contour_points;
        all_contour_points.reserve(cluster_indices.size() * sample_count);
        for (int idx : cluster_indices) {
            const auto& obs = obstacles[idx];
            for (int k = 0; k < sample_count; ++k) {
                double t = 2.0 * M_PI * k / sample_count;
                double x_local = obs.a * std::cos(t);
                double y_local = obs.b * std::sin(t);
                double x_global = obs.center_x
                                  + x_local * std::cos(obs.theta)
                                  - y_local * std::sin(obs.theta);
                double y_global = obs.center_y
                                  + x_local * std::sin(obs.theta)
                                  + y_local * std::cos(obs.theta);
                all_contour_points.emplace_back(
                    static_cast<float>(x_global),
                    static_cast<float>(y_global));
            }
        }
        
        // Step 4.2: 用所有物理轮廓点计算凸包
        std::vector<cv::Point2f> hull;
        cv::convexHull(all_contour_points, hull);
        
        // Step 4.3: 计算最小外接矩形
        cv::RotatedRect min_rect = cv::minAreaRect(hull);
        
        // Step 4.4: min_rect 的宽高已经是纯物理边界的包络，
        // 其一半直接作为新的物理椭圆半轴，不再叠加任何力场偏移。
        double a_new = min_rect.size.width  / 2.0;
        double b_new = min_rect.size.height / 2.0;
        
        if (a_new <= 0.0) a_new = 0.05;
        if (b_new <= 0.0) b_new = 0.05;
        
        // 创建新的融合障碍物
        EllipseObstacle merged_obs;
        merged_obs.id = next_merged_id++;
        merged_obs.center_x = min_rect.center.x;
        merged_obs.center_y = min_rect.center.y;
        merged_obs.a = a_new;
        merged_obs.b = b_new;
        merged_obs.theta = min_rect.angle * M_PI / 180.0;
        merged_obs.last_update_time = ros::Time::now();
        merged_obs.is_merged = true;
        
        for (int idx : cluster_indices) {
            merged_obs.merged_from_ids.push_back(obstacles[idx].id);
        }
        
        final_obstacles.push_back(merged_obs);
    }
    
    ROS_DEBUG_THROTTLE(1.0, "Obstacle Merging: %zu -> %zu", obstacles.size(), final_obstacles.size());
    
    return final_obstacles;
}

// ========== 融合锁定机制 ==========

/**
 * @brief 检查是否应该锁定融合状态（进入反应边界时）
 * 同时计算并锁定绕行方向K
 */
void VectorFieldPlanner::checkAndLockFusion(double x, double y) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    std::lock_guard<std::mutex> obs_guard(obstacle_mutex_);
    
    if (fusion_lock_.is_locked) {
        return;
    }
    
    double current_yaw = getCurrentYaw();
    
    for (const auto& obstacle : obstacles_) {
        double phi_reactive;
        Eigen::Vector2d grad_reactive;
        ellipseFunctionAndGradient(x, y, obstacle, reactive_offset_,
                                  phi_reactive, grad_reactive);
        
        if (phi_reactive < 0.0) {
            fusion_lock_.locked_obstacle_id = obstacle.id;
            fusion_lock_.is_locked = true;
            fusion_lock_.lock_start_time = ros::Time::now();
            
            fusion_lock_.locked_center_x = obstacle.center_x;
            fusion_lock_.locked_center_y = obstacle.center_y;
            fusion_lock_.locked_a = obstacle.a;
            fusion_lock_.locked_b = obstacle.b;
            fusion_lock_.locked_theta = obstacle.theta;
            
            // 计算并锁定绕行方向K（基于极限圆理论）
            fusion_lock_.locked_K = computeKWithLimitCycle(obstacle, x, y, current_yaw);
            
            fusion_lock_.locked_merge_ids.clear();
            fusion_lock_.locked_merge_ids.insert(obstacle.id);
            if (obstacle.is_merged) {
                for (int orig_id : obstacle.merged_from_ids) {
                    fusion_lock_.locked_merge_ids.insert(orig_id);
                }
            }
            
            ROS_INFO("🔒 锁定障碍物 ID:%d, 位置=(%.2f,%.2f), K=%.1f(%s), 融合源:%zu", 
                     obstacle.id, 
                     fusion_lock_.locked_center_x, fusion_lock_.locked_center_y,
                     fusion_lock_.locked_K, 
                     fusion_lock_.locked_K > 0 ? "右绕" : "左绕",
                     fusion_lock_.locked_merge_ids.size());
            break;
        }
    }
}

/**
 * @brief 检查是否应该解锁融合状态
 * 
 * 解锁条件：
 * 1. 锁定的障碍物完全消失（所有源ID都不存在）
 * 2. 车辆离开锁定障碍物的反应边界（phi_reactive >= 0）
 */
void VectorFieldPlanner::checkAndUnlockFusion(double x, double y) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    std::lock_guard<std::mutex> obs_guard(obstacle_mutex_);
    
    if (!fusion_lock_.is_locked) {
        return;
    }
    
    bool any_locked_id_exists = false;
    
    for (const auto& obstacle : obstacles_) {
        if (fusion_lock_.locked_merge_ids.count(obstacle.id) > 0) {
            any_locked_id_exists = true;
            break;
        }
        
        if (obstacle.is_merged) {
            for (int orig_id : obstacle.merged_from_ids) {
                if (fusion_lock_.locked_merge_ids.count(orig_id) > 0) {
                    any_locked_id_exists = true;
                    break;
                }
            }
            if (any_locked_id_exists) break;
        }
    }
    
    // 解锁条件1：所有锁定的源ID都消失了
    if (!any_locked_id_exists) {
        ROS_INFO("🔓 解锁融合障碍物 ID:%d (所有源ID已消失), 锁定时长:%.2fs", 
                 fusion_lock_.locked_obstacle_id,
                 (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
        
        fusion_lock_.is_locked = false;
        fusion_lock_.locked_obstacle_id = -1;
        fusion_lock_.locked_merge_ids.clear();
        return;
    }
    
    // 解锁条件2：车辆离开了锁定障碍物的反应边界
    // 使用锁定时保存的障碍物几何参数来计算
    EllipseObstacle locked_obstacle_temp;
    locked_obstacle_temp.center_x = fusion_lock_.locked_center_x;
    locked_obstacle_temp.center_y = fusion_lock_.locked_center_y;
    locked_obstacle_temp.a = fusion_lock_.locked_a;
    locked_obstacle_temp.b = fusion_lock_.locked_b;
    locked_obstacle_temp.theta = fusion_lock_.locked_theta;
    
    double phi_reactive;
    Eigen::Vector2d grad_reactive;
    ellipseFunctionAndGradient(x, y, locked_obstacle_temp, reactive_offset_,
                              phi_reactive, grad_reactive);
    
    if (phi_reactive >= 0.0) {
        ROS_INFO("🔓 解锁融合障碍物 ID:%d (车辆已离开反应边界, phi=%.3f), 锁定时长:%.2fs", 
                 fusion_lock_.locked_obstacle_id,
                 phi_reactive,
                 (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
        
        fusion_lock_.is_locked = false;
        fusion_lock_.locked_obstacle_id = -1;
        fusion_lock_.locked_merge_ids.clear();
    }
}

// ========== 路径函数 ==========
void VectorFieldPlanner::circlePathFunction(double x, double y,
                                           double& phi_val, Eigen::Vector2d& grad_phi) {
    double dx = x - circle_center_x_;
    double dy = y - circle_center_y_;
    
    phi_val = dx * dx + dy * dy - circle_radius_ * circle_radius_;
    grad_phi << 2.0 * dx, 2.0 * dy;
}

void VectorFieldPlanner::linePathFunction(double x, double y,
                                         double& phi_val, Eigen::Vector2d& grad_phi) {
    // 直线路径：使用标准直线方程 ax + by + d = 0
    // 从起点(x1,y1)到终点(x2,y2)
    double x1 = line_start_x_;
    double y1 = line_start_y_;
    double x2 = line_end_x_;
    double y2 = line_end_y_;
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    // 直线方程系数: ax + by + d = 0
    double a, b, d;
    
    if (std::abs(dx) > epsilon_) {
        // 斜率 m = dy/dx
        // 直线方程: y - y1 = m(x - x1) => mx - y + (y1 - mx1) = 0
        double m = dy / dx;
        a = -m;
        b = 1.0;
        d = m * x1 - y1;
    } else {
        // 垂直线: x = x1
        a = 1.0;
        b = 0.0;
        d = -x1;
    }
    
    // 直线隐式函数值
    phi_val = a * x + b * y + d;
    
    // 梯度（常数）
    grad_phi << a, b;
}

void VectorFieldPlanner::figure8PathFunction(double x, double y,
                                            double& phi_val, Eigen::Vector2d& grad_phi) {
    // 8字形隐式函数: (x^2 + y^2)^2 = a^2(x^2 - y^2)
    // 等价于: (x^2 + y^2)^2 - a^2(x^2 - y^2) = 0
    double a = figure8_scale_;
    double x2 = x * x;
    double y2 = y * y;
    double r2 = x2 + y2;
    
    phi_val = r2 * r2 - a * a * (x2 - y2);
    
    // 梯度计算: ∇φ = [4x(x^2+y^2) - 2a^2*x, 4y(x^2+y^2) + 2a^2*y]
    grad_phi << 4.0 * x * r2 - 2.0 * a * a * x,
                4.0 * y * r2 + 2.0 * a * a * y;
}

// ========== Bump函数 ==========
// 根据论文公式实现的辅助函数f1和f2
// f1(ξ) = { 0,                    φ(ξ) ≤ c
//         { exp(l1/(c-φ(ξ))),    φ(ξ) > c
double VectorFieldPlanner::computeF1(double phi_val) {
    if (phi_val <= c_) {
        return 0.0;
    }
    double denominator = c_ - phi_val;
    if (std::abs(denominator) < epsilon_) {
        return 0.0;  // 避免除零
    }
    return std::exp(l1_ / denominator);
}

// f2(ξ) = { exp(l2/φ(ξ)),    φ(ξ) < 0
//         { 0,                φ(ξ) ≥ 0
double VectorFieldPlanner::computeF2(double phi_val) {
    if (phi_val >= 0.0) {
        return 0.0;
    }
    if (std::abs(phi_val) < epsilon_) {
        return 0.0;  // 避免除零
    }
    return std::exp(l2_ / phi_val);
}

// ⊔Q(ξ) = f1(ξ) / (f1(ξ) + f2(ξ))
// 平滑零入函数：在HQ内为0，在exR内为1，在exQ∩HR内平滑过渡
double VectorFieldPlanner::safeUQ(double phi_val) {
    double f1 = computeF1(phi_val);
    double f2 = computeF2(phi_val);
    
    double sum = f1 + f2;
    if (sum < epsilon_) {
        // 根据论文定义：
        // - 在排斥区域内部 (HQ, φ < c): ⊔Q = 0
        // - 在反应区域外部 (exR, φ > 0): ⊔Q = 1
        if (phi_val <= c_) {
            return 0.0;  // 排斥区域内
        } else {
            return 1.0;  // 反应区域外
        }
    }
    
    double val = f1 / sum;
    return std::min(std::max(val, 0.0), 1.0);
}

// ⊓R(ξ) = f2(ξ) / (f1(ξ) + f2(ξ))
// 平滑零出函数：在HQ内为1，在exR内为0，在exQ∩HR内平滑过渡
double VectorFieldPlanner::safeTR(double phi_val) {
    double f1 = computeF1(phi_val);
    double f2 = computeF2(phi_val);
    
    double sum = f1 + f2;
    if (sum < epsilon_) {
        // 根据论文定义：
        // - 在排斥区域内部 (HQ, φ < c): ⊓R = 1
        // - 在反应区域外部 (exR, φ > 0): ⊓R = 0
        if (phi_val <= c_) {
            return 1.0;  // 排斥区域内
        } else {
            return 0.0;  // 反应区域外
        }
    }
    
    double val = f2 / sum;
    return std::min(std::max(val, 0.0), 1.0);
}

// ========== 辅助函数 ==========
Eigen::Vector2d VectorFieldPlanner::defaultMinVector() {
    return Eigen::Vector2d(min_vector_norm_, 0.0);
}

Eigen::Vector2d VectorFieldPlanner::normalizeWithLimits(
    const Eigen::Vector2d& vec, bool zero_default) {
    
    Eigen::Vector2d default_vec = zero_default ? Eigen::Vector2d::Zero() : defaultMinVector();
    
    if (!vec.allFinite()) {
        return default_vec;
    }
    
    double norm = vec.norm();
    if (norm < epsilon_) {
        return default_vec;
    }
    
    if (norm > max_vector_norm_) {
        return vec * (max_vector_norm_ / norm);
    }
    
    return vec / norm;
}

// ========== 椭圆函数 ==========
void VectorFieldPlanner::ellipseFunctionAndGradient(
    double x, double y, const EllipseObstacle& ellipse,
    double offset, double& phi_val, Eigen::Vector2d& grad) {
    
    double cx = ellipse.center_x;
    double cy = ellipse.center_y;
    double a = ellipse.a + offset;
    double b = ellipse.b + offset;
    double theta = ellipse.theta;
    
    // 旋转变换
    double x_rot = (x - cx) * std::cos(theta) + (y - cy) * std::sin(theta);
    double y_rot = -(x - cx) * std::sin(theta) + (y - cy) * std::cos(theta);
    
    // 椭圆函数值
    phi_val = (x_rot / a) * (x_rot / a) + (y_rot / b) * (y_rot / b) - 1.0;
    
    // 梯度计算
    double dval_dx_rot = 2.0 * x_rot / (a * a);
    double dval_dy_rot = 2.0 * y_rot / (b * b);
    
    // 旋转回原坐标系
    double grad_x = dval_dx_rot * std::cos(theta) - dval_dy_rot * std::sin(theta);
    double grad_y = dval_dx_rot * std::sin(theta) + dval_dy_rot * std::cos(theta);
    
    grad << grad_x, grad_y;
}

// ========== 基于极限圆的方向决策函数 ==========
/**
 * @brief 计算到椭圆边界的物理距离（Sampson Error 近似）
 * 
 * 基于 Sampson Error 公式：d ≈ |f(x,y)| / ||∇f(x,y)||
 * 其中 f(x,y) 是椭圆隐函数（phi 值）
 * 
 * 这个距离用于极限圆安全裕度的计算，以决定绕行方向
 * 
 * @param x 查询点x坐标
 * @param y 查询点y坐标
 * @param obs 椭圆障碍物
 * @return 到物理边界（无偏移）的近似欧氏距离
 */
double VectorFieldPlanner::calculatePhysicalDistance(double x, double y, 
                                                     const EllipseObstacle& obs) {
    double phi;
    Eigen::Vector2d grad;
    
    // 使用 offset=0 计算到物理边界的距离
    ellipseFunctionAndGradient(x, y, obs, 0.0, phi, grad);
    
    double grad_norm = grad.norm();
    if (grad_norm < epsilon_) {
        return 0.0;  // 在中心附近，返回0
    }
    
    // Sampson Error: d ≈ |φ| / ||∇φ||
    return std::abs(phi) / grad_norm;
}

/**
 * @brief 基于极限圆理论计算绕行方向K
 * 
 * 基于论文 "Distributed Safe Cooperative Vector Field" 的极限圆概念：
 * 1. 左转极限圆圆心: C_L = (x - R*sin(θ), y + R*cos(θ))
 * 2. 右转极限圆圆心: C_R = (x + R*sin(θ), y - R*cos(θ))
 * 3. 计算两个圆心到障碍物边界的物理距离
 * 4. 选择安全裕度更大的方向
 * 
 * 安全裕度 = 到障碍物的距离 - 最小转弯半径
 * 
 * @param obs 椭圆障碍物
 * @param x 当前车辆x坐标
 * @param y 当前车辆y坐标
 * @param current_yaw 当前车辆朝向
 * @return K值：+1.0 表示逆时针（左转）, -1.0 表示顺时针（右转）
 */
double VectorFieldPlanner::computeKWithLimitCycle(const EllipseObstacle& obs,
                                                   double x, double y, 
                                                   double current_yaw) {
    // 计算左转和右转极限圆的圆心位置
    double R = min_turning_radius_;
    double sin_yaw = std::sin(current_yaw);
    double cos_yaw = std::cos(current_yaw);
    
    // 左转极限圆圆心（车辆左侧一个转弯半径处）
    double left_cx = x - R * sin_yaw;
    double left_cy = y + R * cos_yaw;
    
    // 右转极限圆圆心（车辆右侧一个转弯半径处）
    double right_cx = x + R * sin_yaw;
    double right_cy = y - R * cos_yaw;
    
    // 计算两个圆心到障碍物的物理距离
    double dist_left = calculatePhysicalDistance(left_cx, left_cy, obs);
    double dist_right = calculatePhysicalDistance(right_cx, right_cy, obs);
    
    // 安全裕度 = 距离 - 转弯半径
    double margin_left = dist_left - R;
    double margin_right = dist_right - R;
    
    // 选择安全裕度更大的方向
    // 注意：E矩阵是顺时针旋转90度，所以：
    // K = +1.0: 顺时针（右转）
    // K = -1.0: 逆时针（左转）
    if (margin_left >= margin_right) {
        return -1.0;  // 左转更安全，使用负K
    } else {
        return 1.0;   // 右转更安全，使用正K
    }
}

/**
 * @brief 获取绕行方向K（优先使用锁定值）
 * 
 * 如果当前障碍物是锁定的障碍物，返回锁定时计算的K值
 * 否则使用极限圆理论实时计算K值
 */
double VectorFieldPlanner::getLockedOrComputeK(const EllipseObstacle& obs,
                                                double x, double y, 
                                                double current_yaw) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    
    if (fusion_lock_.is_locked) {
        // 检查当前障碍物是否是锁定的目标
        if (obs.id == fusion_lock_.locked_obstacle_id) {
            return fusion_lock_.locked_K;
        }
        
        // 检查融合ID
        if (obs.is_merged) {
            for (int orig_id : obs.merged_from_ids) {
                if (fusion_lock_.locked_merge_ids.count(orig_id) > 0) {
                    return fusion_lock_.locked_K;
                }
            }
        }
        
        // 检查当前障碍物ID是否在锁定的合并ID集合中
        if (fusion_lock_.locked_merge_ids.count(obs.id) > 0) {
            return fusion_lock_.locked_K;
        }
    }
    
    // 未锁定或不是锁定的障碍物，使用极限圆理论计算新的K
    return computeKWithLimitCycle(obs, x, y, current_yaw);
}

// ========== 向量场计算 ==========
Eigen::Vector2d VectorFieldPlanner::pathTrackingVectorField(double x, double y) {
    double phi;
    Eigen::Vector2d grad_phi;
    
    // 根据路径类型计算phi和梯度
    if (path_type_ == "circle") {
        circlePathFunction(x, y, phi, grad_phi);
    } else if (path_type_ == "line") {
        linePathFunction(x, y, phi, grad_phi);
    } else if (path_type_ == "figure8") {
        figure8PathFunction(x, y, phi, grad_phi);
    } else {
        ROS_WARN_THROTTLE(5.0, "不支持的路径类型: %s", path_type_.c_str());
        return defaultMinVector();
    }
    
    double grad_norm = grad_phi.norm();
    if (grad_norm < epsilon_) {
        return defaultMinVector();
    }
    
    // 统一的向量场公式：χ_P = K·E·∇φ - k_p·φ·∇φ
    double direction_sign = 1.0;
    
    if (path_type_ == "circle") {
        direction_sign = (circle_direction_ == "ccw") ? 1.0 : -1.0;
    } else if (path_type_ == "line") {
        // 线段路径：计算正确的方向（参考MATLAB代码）
        // 法向量 = [a, b] = grad_phi
        // 切线向量 = [-b, a] (旋转90度)
        Eigen::Vector2d tangent_vector(-grad_phi(1), grad_phi(0));
        
        // 起点到终点向量
        Eigen::Vector2d start_to_goal(line_end_x_ - line_start_x_, 
                                     line_end_y_ - line_start_y_);
        
        // 投影到切线方向
        double projection = start_to_goal.dot(tangent_vector);
        direction_sign = (projection >= 0) ? 1.0 : -1.0;
        
        // 如果用户指定了backward，则反向
        if (line_direction_ == "backward") {
            direction_sign = -direction_sign;
        }
    } else if (path_type_ == "figure8") {
        direction_sign = (figure8_direction_ == "ccw") ? 1.0 : -1.0;
    }
    
    Eigen::Vector2d chi_P = direction_sign * (E_ * grad_phi) - k_p_ * phi * grad_phi;
    
    if (!chi_P.allFinite()) {
        return defaultMinVector();
    }
    
    return chi_P;
}

Eigen::Vector2d VectorFieldPlanner::reactiveVectorField(double x, double y) {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    
    if (obstacles_.empty()) {
        return Eigen::Vector2d::Zero();
    }
    
    // 找到最近的障碍物
    double min_distance = std::numeric_limits<double>::infinity();
    int nearest_index = -1;
    
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        double phi_reactive;
        Eigen::Vector2d grad_reactive;
        ellipseFunctionAndGradient(x, y, obstacles_[i], reactive_offset_,
                                  phi_reactive, grad_reactive);
        
        if (!std::isfinite(phi_reactive)) continue;
        
        if (phi_reactive < min_distance) {
            min_distance = phi_reactive;
            nearest_index = static_cast<int>(i);
        }
    }
    
    if (nearest_index < 0) {
        return Eigen::Vector2d::Zero();
    }
    
    const auto& nearest_obstacle = obstacles_[nearest_index];
    double phi_reactive;
    Eigen::Vector2d grad_phi_reactive;
    ellipseFunctionAndGradient(x, y, nearest_obstacle, reactive_offset_,
                              phi_reactive, grad_phi_reactive);
    
    if (!grad_phi_reactive.allFinite()) {
        return Eigen::Vector2d::Zero();
    }
    
    double grad_norm = grad_phi_reactive.norm();
    if (grad_norm < epsilon_) {
        return Eigen::Vector2d::Zero();
    }
    
    // 使用锁定的K值（如果已锁定）或计算新的K值
    double current_yaw = getCurrentYaw();
    double K = getLockedOrComputeK(nearest_obstacle, x, y, current_yaw);
    
    // ========== 基于物理距离的反应向量场（物理一致性修正）==========
    // 1. 计算单位法向量 n = ∇φ / ||∇φ||
    Eigen::Vector2d n = grad_phi_reactive / grad_norm;
    
    // 2. 计算物理距离（Sampson Error）: d = φ / ||∇φ||
    //    注意：在反应边界内 φ < 0，所以 d 也是负值
    double dist_physical = phi_reactive / grad_norm;
    
    // 3. 新的向量场公式: χ_R = K · (E · n) - k_r · d · n
    //    第一项 K·(E·n): 切向绕行速度，大小恒定，只提供方向
    //    第二项 -k_r·d·n: 法向斥力速度，当 d<0 时产生向外推力
    //    推力大小与物理侵入距离成正比，不受障碍物尺寸参数影响
    Eigen::Vector2d chi_R = K * (E_ * n) - k_r_ * dist_physical * n;
    
    if (!chi_R.allFinite()) {
        return Eigen::Vector2d::Zero();
    }
    
    return chi_R;
}

Eigen::Vector2d VectorFieldPlanner::compositeVectorField(double x, double y) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return defaultMinVector();
    }
    
    // 获取当前车朝向，用于极限圆方向决策
    double current_yaw = getCurrentYaw();
    
    // 检查并更新融合锁定状态
    checkAndLockFusion(x, y);
    checkAndUnlockFusion(x, y);
    
    // 预计算路径跟踪向量场
    Eigen::Vector2d chi_P = pathTrackingVectorField(x, y);
    Eigen::Vector2d chi_P_hat = normalizeWithLimits(chi_P);
    
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    
    if (obstacles_.empty()) {
        return chi_P_hat;
    }
    
    // 检查是否在排斥区域内
    bool in_repulsive_zone = false;
    int nearest_index = -1;
    double min_phi_repulsive = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        double phi_repulsive;
        Eigen::Vector2d grad_phi_repulsive;
        ellipseFunctionAndGradient(x, y, obstacles_[i], repulsive_offset_,
                                  phi_repulsive, grad_phi_repulsive);
        
        if (!std::isfinite(phi_repulsive)) continue;
        
        if (phi_repulsive < min_phi_repulsive) {
            min_phi_repulsive = phi_repulsive;
            nearest_index = static_cast<int>(i);
        }
        
        if (phi_repulsive < 0.0) {
            in_repulsive_zone = true;
            break;
        }
    }
    
    // 1. 排斥区域内：只使用反应向量场
    if (in_repulsive_zone && nearest_index >= 0) {
        const auto& nearest_obstacle = obstacles_[nearest_index];
        
        double phi_reactive;
        Eigen::Vector2d grad_phi_reactive;
        ellipseFunctionAndGradient(x, y, nearest_obstacle, reactive_offset_,
                                  phi_reactive, grad_phi_reactive);
        
        if (grad_phi_reactive.allFinite()) {
            double grad_norm = grad_phi_reactive.norm();
            if (grad_norm >= epsilon_) {
                // 使用锁定的K值（如果已锁定）
                double K = getLockedOrComputeK(nearest_obstacle, x, y, current_yaw);
                
                // 基于物理距离的反应向量场
                Eigen::Vector2d n = grad_phi_reactive / grad_norm;
                double dist_physical = phi_reactive / grad_norm;
                Eigen::Vector2d chi_R = K * (E_ * n) - k_r_ * dist_physical * n;
                
                Eigen::Vector2d chi_R_hat = normalizeWithLimits(chi_R, true);
                return chi_R_hat;
            }
        }
    }
    
    // 2. 混合区域：使用 U_Q_product 和 T_R_sum 进行复合
    double U_Q_product = 1.0;
    Eigen::Vector2d T_R_sum = Eigen::Vector2d::Zero();
    
    for (const auto& obstacle : obstacles_) {
        double phi_repulsive, phi_reactive;
        Eigen::Vector2d grad_phi_repulsive, grad_phi_reactive;
        
        ellipseFunctionAndGradient(x, y, obstacle, repulsive_offset_,
                                  phi_repulsive, grad_phi_repulsive);
        ellipseFunctionAndGradient(x, y, obstacle, reactive_offset_,
                                  phi_reactive, grad_phi_reactive);
        
        if (!std::isfinite(phi_repulsive) || !std::isfinite(phi_reactive)) {
            continue;
        }
        
        double U_Q_val = safeUQ(phi_repulsive);
        double T_R_val = safeTR(phi_reactive);
        
        // 反应向量场（基于物理距离）
        Eigen::Vector2d chi_R = Eigen::Vector2d::Zero();
        if (grad_phi_reactive.allFinite()) {
            double grad_norm = grad_phi_reactive.norm();
            if (grad_norm >= epsilon_) {
                // 使用锁定的K值（如果已锁定）
                double K = getLockedOrComputeK(obstacle, x, y, current_yaw);
                
                // 基于物理距离的反应向量场
                Eigen::Vector2d n = grad_phi_reactive / grad_norm;
                double dist_physical = phi_reactive / grad_norm;
                chi_R = K * (E_ * n) - k_r_ * dist_physical * n;
            }
        }
        
        Eigen::Vector2d chi_R_hat = normalizeWithLimits(chi_R, true);
        
        U_Q_product *= U_Q_val;
        T_R_sum += T_R_val * chi_R_hat;
    }
    
    if (!std::isfinite(U_Q_product)) {
        U_Q_product = 1.0;
    }
    
    // 合成最终导引向量: χ_c = U_Q_product * χ_P_hat + T_R_sum
    Eigen::Vector2d chi_c = U_Q_product * chi_P_hat + T_R_sum;
    
    if (!chi_c.allFinite()) {
        return defaultMinVector();
    }
    
    double chi_norm = chi_c.norm();
    if (chi_norm > max_vector_norm_) {
        chi_c = chi_c * (max_vector_norm_ / chi_norm);
    } else if (chi_norm < min_vector_norm_) {
        chi_c = defaultMinVector();
    }
    
    return chi_c;
}

// ========== Ackermann控制 ==========
geometry_msgs::Twist VectorFieldPlanner::vectorToAckermann(
    const Eigen::Vector2d& chi_c, double current_yaw) {
    
    geometry_msgs::Twist cmd;
    
    double chi_norm = chi_c.norm();
    if (std::isnan(chi_norm) || chi_norm < 1e-6) {
        ROS_WARN_THROTTLE(2.0, "向量场无效或太小: norm=%.6f", chi_norm);
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }
    
    Eigen::Vector2d chi_c_normalized = chi_c / chi_norm;
    
    double desired_yaw = std::atan2(chi_c_normalized(1), chi_c_normalized(0));
    
    double yaw_error = desired_yaw - current_yaw;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
    
    if (yaw_error_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(yaw_error);
        yaw_error_pub_.publish(msg);
    }
    
    double linear_vel = max_vector_norm_;
    
    // double steering_angle = steering_gain_ * yaw_error;
    double steering_angle = steering_gain_ * yaw_error;
    steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));
    
    // 发布控制调试信息
    // if (control_debug_pub_.getNumSubscribers() > 0) {
    //     bringup::ControlDebug debug_msg;
    //     debug_msg.header.stamp = ros::Time::now();
    //     debug_msg.header.frame_id = "map";
        
    //     // 向量场信息
    //     debug_msg.vector_x = chi_c(0);
    //     debug_msg.vector_y = chi_c(1);
    //     debug_msg.vector_norm = chi_norm;
        
    //     // 角度信息（弧度）
    //     debug_msg.current_yaw = current_yaw;
    //     debug_msg.desired_yaw = desired_yaw;
    //     debug_msg.yaw_error = yaw_error;
        
    //     // 控制输出
    //     debug_msg.steering_angle = steering_angle;
    //     debug_msg.linear_velocity = linear_vel;
        
    //     // 角度信息（度）
    //     debug_msg.current_yaw_deg = current_yaw * 180.0 / M_PI;
    //     debug_msg.desired_yaw_deg = desired_yaw * 180.0 / M_PI;
    //     debug_msg.yaw_error_deg = yaw_error * 180.0 / M_PI;
    //     debug_msg.steering_angle_deg = steering_angle * 180.0 / M_PI;
        
    //     // 额外调试信息
    //     if (has_velocity_) {
    //         std::lock_guard<std::mutex> lock(pose_mutex_);
    //         double vx = current_velocity_.linear.x;
    //         double vy = current_velocity_.linear.y;
    //         debug_msg.speed_magnitude = std::sqrt(vx * vx + vy * vy);
    //         debug_msg.speed_direction = std::atan2(vy, vx);
    //     } else {
    //         debug_msg.speed_magnitude = 0.0;
    //         debug_msg.speed_direction = 0.0;
    //     }
        
    //     control_debug_pub_.publish(debug_msg);
    // }
    
    // ROS_INFO_THROTTLE(0.5,
    //     "向量=[%.3f,%.3f], 向量场方向=%.1f°, 当前朝向=%.1f°, 角度误差=%.1f°, 舵机角度=%.1f°",
    //     chi_c(0), chi_c(1), desired_yaw * 180.0 / M_PI, current_yaw * 180.0 / M_PI,
    //     yaw_error * 180.0 / M_PI, steering_angle * 180.0 / M_PI);
    
    cmd.linear.x = linear_vel;
    cmd.angular.z = steering_angle;
    
    return cmd;
}

double VectorFieldPlanner::getCurrentYaw() {
    // 使用TF树获取车体在map坐标系中的朝向（最准确的方法）
    try {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
        
        double yaw = tf::getYaw(transform.getRotation());
        
        // 调试信息：对比里程计中的朝向
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (has_pose_) {
            tf::Quaternion q_odom(current_pose_.orientation.x,
                                 current_pose_.orientation.y,
                                 current_pose_.orientation.z,
                                 current_pose_.orientation.w);
            double odom_yaw = tf::getYaw(q_odom);
            
            ROS_DEBUG_THROTTLE(2.0, "🔍 TF车体朝向=%.1f°, 里程计朝向=%.1f°, 差异=%.1f°",
                            yaw * 180.0 / M_PI, odom_yaw * 180.0 / M_PI, 
                            (yaw - odom_yaw) * 180.0 / M_PI);
        }
        
        // 速度方向对比
        if (has_velocity_) {
            double vx = current_velocity_.linear.x;
            double vy = current_velocity_.linear.y;
            double v_speed = std::sqrt(vx * vx + vy * vy);
            
            if (v_speed > 0.05) {
                double v_yaw = std::atan2(vy, vx);
                ROS_DEBUG_THROTTLE(2.0, "🚗 车体朝向=%.1f°, 速度方向=%.1f°, 速度=%.2fm/s",
                                yaw * 180.0 / M_PI, v_yaw * 180.0 / M_PI, v_speed);
            }
        }
        
        return yaw;
        
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "获取车体朝向失败: %s，使用里程计备用方案", ex.what());
        
        // 备用方案：使用里程计数据
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_pose_) {
            return 0.0;
        }
        
        tf::Quaternion q(current_pose_.orientation.x,
                        current_pose_.orientation.y,
                        current_pose_.orientation.z,
                        current_pose_.orientation.w);
        return tf::getYaw(q);
    }
}


void VectorFieldPlanner::computeLocalVectorField() {
    ros::Time current_time = ros::Time::now();
    
    if ((current_time - last_field_update_time_).toSec() < field_update_interval_) {
        return;
    }
    
    last_field_update_time_ = current_time;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    double half_range = field_range_ / 2.0;
    std::map<std::pair<int, int>, Eigen::Vector2d> new_field;
    
    for (double x = field_center_x_ - half_range; 
         x < field_center_x_ + half_range; 
         x += field_resolution_) {
        for (double y = field_center_y_ - half_range; 
             y < field_center_y_ + half_range; 
             y += field_resolution_) {
            try {
                Eigen::Vector2d chi_c = compositeVectorField(x, y);
                auto grid_key = positionToGridKey(x, y);
                new_field[grid_key] = chi_c;
            } catch (std::exception& e) {
                ROS_DEBUG("向量场计算失败 at (%.2f, %.2f): %s", x, y, e.what());
                continue;
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    
    if (field_compute_time_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(elapsed_time);
        field_compute_time_pub_.publish(msg);
    }
    
    cached_vector_field_ = new_field;
}

std::pair<int, int> VectorFieldPlanner::positionToGridKey(double x, double y) {
    int grid_x = static_cast<int>(std::round(x / field_resolution_));
    int grid_y = static_cast<int>(std::round(y / field_resolution_));
    return std::make_pair(grid_x, grid_y);
}

Eigen::Vector2d VectorFieldPlanner::getVectorAtPosition(double x, double y) {
    if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y)) {
        ROS_WARN("无效的位置输入: x=%.2f, y=%.2f", x, y);
        return Eigen::Vector2d::Zero();
    }
    
    // 直接计算当前位置的向量场，避免预计算整个网格
    Eigen::Vector2d chi = compositeVectorField(x, y);
    if (!chi.allFinite()) {
        return Eigen::Vector2d::Zero();
    }
    return chi;
}

// ========== 局部路径生成 ==========
nav_msgs::Path VectorFieldPlanner::generateLocalPath(double start_x, double start_y) {
    nav_msgs::Path local_path;
    local_path.header.frame_id = "map";
    local_path.header.stamp = ros::Time::now();
    
    double x = start_x;
    double y = start_y;
    double accumulated_distance = 0.0;
    
    while (accumulated_distance < local_path_length_) {
        geometry_msgs::PoseStamped pose;
        pose.header = local_path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        Eigen::Vector2d chi = getVectorAtPosition(x, y);
        double chi_norm = chi.norm();
        
        if (chi_norm < min_vector_norm_) {
            break;
        }
        
        double yaw = std::atan2(chi(1), chi(0));
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        local_path.poses.push_back(pose);
        
        double step = std::min(local_path_step_, local_path_length_ - accumulated_distance);
        Eigen::Vector2d direction = chi / chi_norm;
        x += direction(0) * step;
        y += direction(1) * step;
        accumulated_distance += step;
    }
    
    return local_path;
}

} // namespace vector_field_planner
