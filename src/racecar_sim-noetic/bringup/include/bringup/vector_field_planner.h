#ifndef VECTOR_FIELD_PLANNER_H
#define VECTOR_FIELD_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <map>
#include <set>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>

// OpenCV for OBB overlap detection in obstacle merging (geometry only, no visualization)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace vector_field_planner {

/**
 * @brief 椭圆障碍物数据结构
 */
struct EllipseObstacle {
    int id;               // 障碍物ID
    double center_x;      // 中心X坐标
    double center_y;      // 中心Y坐标
    double a;             // 半长轴
    double b;             // 半短轴
    double theta;         // 旋转角度（弧度）
    ros::Time last_update_time;  // 最后更新时间
    bool is_merged;       // 标记是否为融合后生成的障碍物
    std::vector<int> merged_from_ids;  // 记录融合来源ID
    
    // 更新构造函数
    EllipseObstacle(int obs_id, double cx, double cy, double semi_major, double semi_minor, double orientation)
        : id(obs_id), center_x(cx), center_y(cy), a(semi_major / 2.0), b(semi_minor / 2.0), theta(orientation),
          last_update_time(ros::Time::now()), is_merged(false) {}
          
    // 为了兼容旧代码的默认构造
    EllipseObstacle() : id(-1), center_x(0), center_y(0), a(0), b(0), theta(0), is_merged(false) {}
};

/**
 * @brief 向量场局部路径规划器（C++版本）
 * 
 * 基于引导向量场的避障方法，使用极限圆进行动态方向决策
 * 参考论文：《Distributed Safe Cooperative Vector Field》
 */
class VectorFieldPlanner {
public:
    VectorFieldPlanner();
    ~VectorFieldPlanner() = default;
    
    /**
     * @brief 主控制循环
     */
    void run();

private:
    // ========== ROS相关 ==========
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_odom_sub_;
    ros::Subscriber obstacles_sub_;
    
    // 发布器
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_plan_pub_;
    ros::Publisher vector_field_pub_;
    ros::Publisher path_field_pub_;
    ros::Publisher reactive_field_pub_;
    ros::Publisher obstacle_boundaries_pub_;
    ros::Publisher yaw_error_pub_;
    ros::Publisher current_vector_pub_;
    ros::Publisher circle_path_pub_;
    ros::Publisher field_compute_time_pub_;
    ros::Publisher control_query_time_pub_;
    ros::Publisher viz_time_pub_;
    ros::Publisher control_debug_pub_;
    ros::Publisher step_times_pub_;
    ros::Publisher bypass_direction_pub_;  // 发布绕行方向信息给 OpenCV 可视化
    
    // TF监听器
    tf::TransformListener tf_listener_;
    
    // ========== 参数配置 ==========
    // 向量场增益参数
    double k_p_;                    // 路径跟踪增益
    double k_r_;                    // 反应向量场增益
    
    // Bump函数参数
    double l1_;
    double l2_;
    double bump_transition_width_;  // 保留用于兼容，但不再使用
    double c_;  // 论文中的常数c，定义排斥边界
    
    // 边界偏移量
    double repulsive_offset_;
    double reactive_offset_;
    
    // 数值稳定性参数
    double epsilon_;
    double max_vector_norm_;
    double min_vector_norm_;
    Eigen::Matrix2d E_;             // 旋转矩阵
    
    // 控制频率
    double control_rate_;
    
    // 目标点容差
    double goal_tolerance_;
    
    // Ackermann参数
    double wheelbase_;
    double max_steering_angle_;
    double steering_gain_;
    double min_turning_radius_;     // 最小转弯半径 R = L / tan(delta_max)
    
    // 前瞻距离参数
    double lookahead_distance_;  // 车头前瞻距离
    
    // 路径参数
    std::string path_type_;
    
    // 圆形路径参数
    double circle_center_x_;
    double circle_center_y_;
    double circle_radius_;
    std::string circle_direction_;
    
    // 线段路径参数
    double line_start_x_;
    double line_start_y_;
    double line_end_x_;
    double line_end_y_;
    std::string line_direction_;
    
    // 8字形路径参数
    double figure8_scale_;
    std::string figure8_direction_;
    
    // 向量场计算和可视化参数
    double field_center_x_;
    double field_center_y_;
    double field_range_;
    double field_resolution_;
    double viz_frequency_;
    ros::Time last_viz_time_;
    
    // 局部路径生成参数
    double local_path_length_;
    double local_path_step_;
    
    // 障碍物管理参数
    double obstacle_association_threshold_;
    double obstacle_duration_time_;  // 障碍物保留时间（秒）
    
    // ========== 状态变量 ==========
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_;
    bool has_pose_;
    bool has_velocity_;
    
    std::vector<EllipseObstacle> obstacles_;
    
    // ========== 融合锁定机制（含方向锁定） ==========
    // 锁定条件：进入反应边界时锁定融合状态和绕行方向
    // 锁定内容：融合的障碍物ID集合 + 绕行方向K
    struct FusionLock {
        int locked_obstacle_id;         // 锁定的障碍物ID（融合后的ID）
        std::set<int> locked_merge_ids; // 锁定时该障碍物包含的所有原始ID
        bool is_locked;                 // 是否处于锁定状态
        ros::Time lock_start_time;      // 锁定开始时间
        
        // 锁定的障碍物形状和位置参数（全局坐标）
        double locked_center_x;
        double locked_center_y;
        double locked_a;
        double locked_b;
        double locked_theta;
        
        // 锁定的绕行方向（进入反应边界时计算一次）
        double locked_K;                // +1.0=左绕(逆时针), -1.0=右绕(顺时针)
        
        FusionLock() : locked_obstacle_id(-1), is_locked(false),
                       locked_center_x(0), locked_center_y(0), 
                       locked_a(0), locked_b(0), locked_theta(0),
                       locked_K(1.0) {}
    };
    FusionLock fusion_lock_;
    std::mutex lock_mutex_;
    
    // 线程锁
    std::mutex obstacle_mutex_;
    std::mutex pose_mutex_;
    
    // 缓存的向量场数据
    std::map<std::pair<int, int>, Eigen::Vector2d> cached_vector_field_;
    ros::Time last_field_update_time_;
    double field_update_interval_;
    
    // ========== 回调函数 ==========
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstaclesCallback(const std_msgs::String::ConstPtr& msg);
    
    // ========== 障碍物融合 ==========
    /**
     * @brief 基于反应边界OBB的障碍物融合
     * @param obstacles 输入的原始障碍物列表
     * @return 融合后的障碍物列表
     */
    std::vector<EllipseObstacle> mergeObstacles(const std::vector<EllipseObstacle>& obstacles);
    
    /**
     * @brief 检测两个旋转矩形是否重叠
     */
    bool checkOBBOverlap(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2);
    
    /**
     * @brief 构建障碍物的排斥边界OBB
     */
    cv::RotatedRect buildRepulsiveOBB(const EllipseObstacle& obstacle);
    
    // ========== 融合锁定机制 ==========
    /**
     * @brief 检查是否应该锁定融合状态（进入反应边界时）
     */
    void checkAndLockFusion(double x, double y);
    
    /**
     * @brief 检查是否应该解锁融合状态
     */
    void checkAndUnlockFusion(double x, double y);
    
    // ========== 路径函数 ==========
    void circlePathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi);
    void linePathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi);
    void figure8PathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi);
    
    // ========== 向量场计算 ==========
    // 论文中的辅助函数
    double computeF1(double phi_val);  // f1(ξ)
    double computeF2(double phi_val);  // f2(ξ)
    
    // 平滑bump函数
    double safeUQ(double phi_val);  // ⊔Q(ξ): 平滑零入函数
    double safeTR(double phi_val);  // ⊓R(ξ): 平滑零出函数
    
    Eigen::Vector2d defaultMinVector();
    Eigen::Vector2d normalizeWithLimits(const Eigen::Vector2d& vec, bool zero_default = false);
    
    /**
     * @brief 椭圆势场函数及其梯度
     */
    void ellipseFunctionAndGradient(double x, double y, const EllipseObstacle& ellipse, 
                                   double offset, double& phi_val, Eigen::Vector2d& grad);
    
    /**
     * @brief 计算物理距离（Sampson Error / 一阶梯度归一化）
     * 
     * 数学原理: d ≈ |f(x,y)| / ||∇f(x,y)||
     * 其中 f(x,y) 是椭圆的隐式函数
     * 
     * @param x 查询点x坐标
     * @param y 查询点y坐标
     * @param obs 椭圆障碍物
     * @return 点到椭圆物理边界的近似欧氏距离
     */
    double calculatePhysicalDistance(double x, double y, const EllipseObstacle& obs);
    
    /**
     * @brief 基于极限圆的绕行方向决策
     * 
     * 算法原理：
     * 1. 计算左/右极限圆圆心
     * 2. 计算各极限圆到障碍物的安全余量 δ
     * 3. 选择安全余量更大的一侧
     * 
     * @param obs 障碍物
     * @param x 当前位置x
     * @param y 当前位置y
     * @param current_yaw 当前朝向
     * @return K值：1.0(左绕) 或 -1.0(右绕)
     */
    double computeKWithLimitCycle(const EllipseObstacle& obs, double x, double y, 
                                  double current_yaw);
    
    /**
     * @brief 获取绕行方向K（优先使用锁定值）
     * 
     * 如果当前障碍物是锁定的障碍物，返回锁定时计算的K值
     * 否则使用极限圆理论实时计算K值
     */
    double getLockedOrComputeK(const EllipseObstacle& obs, double x, double y, 
                               double current_yaw);
    
    Eigen::Vector2d pathTrackingVectorField(double x, double y);
    
    /**
     * @brief 反应向量场计算（基于极限圆的动态方向决策）
     */
    Eigen::Vector2d reactiveVectorField(double x, double y);
    
    /**
     * @brief 复合向量场计算
     */
    Eigen::Vector2d compositeVectorField(double x, double y);
    
    geometry_msgs::Twist vectorToAckermann(const Eigen::Vector2d& chi_c, double current_yaw);
    double getCurrentYaw();
    
    // ========== 向量场缓存和查询 ==========
    void computeLocalVectorField();
    Eigen::Vector2d getVectorAtPosition(double x, double y);
    
    // ========== 路径生成 ==========
    nav_msgs::Path generateLocalPath(double start_x, double start_y);
    
    // ========== 可视化 ==========
    void visualizeCirclePath();
    void visualizeLinePath();
    void visualizeFigure8Path();
    void visualizeObstacleBoundaries();
    void visualizeCurrentVector(double x, double y, const Eigen::Vector2d& chi_c, double current_yaw);
    void visualizeVectorField();
    
    // ========== 辅助函数 ==========
    bool checkOnPath();
    std::pair<int, int> positionToGridKey(double x, double y);
};

} // namespace vector_field_planner

#endif // VECTOR_FIELD_PLANNER_H
