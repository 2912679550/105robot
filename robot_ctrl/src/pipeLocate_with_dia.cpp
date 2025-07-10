#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

class PipeOdometryConverter {
public:
    struct Pose {
        Eigen::Vector3d position;      // 世界坐标系中的位置 (x, y, z)
        Eigen::Quaterniond orientation; // 世界坐标系中的姿态
        double timestamp;               // 时间戳
    };

    // 输出数据结构
    struct PipeOdometry {
        double axial_displacement = 0.0; // 轴向位移 (从起点开始的累积)
        double circumferential_angle = 0.0; // 周向角度 (弧度，-π 到 π)
        Eigen::Vector3d local_axis;     // 当前估计的局部轴向 (单位向量)
        Eigen::Vector3d pipe_center;    // 当前管道中心点位置
        double timestamp;
    };

    /**
     * 构造函数
     * @param window_size 滑动窗口大小
     * @param pipe_diameter 管道直径
     * @param gravity_dir 重力方向在世界坐标系中的向量
     */
    PipeOdometryConverter(int window_size = 10, 
                          double pipe_diameter = 0.0,
                          const Eigen::Vector3d& gravity_dir = Eigen::Vector3d(0, 0, -1))
        : window_size_(window_size), 
          pipe_diameter_(pipe_diameter),
          pipe_radius_(pipe_diameter/2.0),
          gravity_world_(gravity_dir.normalized()) {
        
        if (window_size < 3) window_size_ = 3;
    }

    /**
     * 处理新位姿并转换为管道里程计
     * @param new_pose 新的位姿输入
     * @return 管道里程计输出
     */
    PipeOdometry processPose(const Pose& new_pose) {
        // 添加到滑动窗口
        pose_window_.push_back(new_pose);
        if (pose_window_.size() > window_size_) {
            pose_window_.pop_front();
        }

        // 初始化检查
        if (!is_initialized_) {
            initialize(new_pose);
            return current_output_; // 返回初始值
        }

        // 1. 估计局部轴向和管道中心
        estimateLocalAxisAndCenter();

        // 2. 计算轴向位移
        Eigen::Vector3d delta_pos = new_pose.position - last_pose_.position;
        double delta_s = delta_pos.dot(current_output_.local_axis);
        axial_displacement_ += delta_s;
        current_output_.axial_displacement = axial_displacement_;

        // 3. 计算周向角度
        current_output_.circumferential_angle = calculateCircumferentialAngle(new_pose);

        // 4. 更新状态
        last_pose_ = new_pose;
        current_output_.timestamp = new_pose.timestamp;

        return current_output_;
    }

    // 设置管道直径（可在运行时更新）
    void setPipeDiameter(double diameter) {
        pipe_diameter_ = diameter;
        pipe_radius_ = diameter / 2.0;
    }

private:
    // 初始化参考系
    void initialize(const Pose& init_pose) {
        last_pose_ = init_pose;
        
        // 初始轴向估计
        if (pose_window_.size() >= 2) {
            Eigen::Vector3d init_dir = (pose_window_.back().position - pose_window_.front().position).normalized();
            current_output_.local_axis = init_dir;
        } else {
            Eigen::Vector3d forward_vec(1, 0, 0);
            current_output_.local_axis = init_pose.orientation * forward_vec;
        }
        
        // 初始管道中心估计
        current_output_.pipe_center = init_pose.position;
        
        // 计算初始参考方向
        Eigen::Vector3d gravity_current = init_pose.orientation * gravity_world_;
        ref_direction_ = projectToSectionPlane(gravity_current, current_output_.local_axis);
        ref_direction_.normalize();
        
        // 如果直径已知，使用直径约束优化初始位置
        if (pipe_diameter_ > 0) {
            // 初始投影点
            Eigen::Vector3d proj_point = projectToSectionPlane(init_pose.position, current_output_.local_axis);
            // 初始方向向量（从中心指向机器人）
            Eigen::Vector3d radial_dir = (init_pose.position - proj_point).normalized();
            // 存储初始径向方向作为参考
            initial_radial_dir_ = radial_dir;
        }
        
        axial_displacement_ = 0.0;
        current_output_.circumferential_angle = 0.0;
        current_output_.axial_displacement = 0.0;
        current_output_.timestamp = init_pose.timestamp;
        
        is_initialized_ = true;
    }

    // 估计局部轴向和管道中心（使用直径约束）
    void estimateLocalAxisAndCenter() {
        // 方法1：使用PCA估计初始轴向
        Eigen::MatrixXd points(pose_window_.size(), 3);
        int i = 0;
        for (const auto& pose : pose_window_) {
            points.row(i++) = pose.position.transpose();
        }
        
        Eigen::Vector3d centroid = points.colwise().mean();
        Eigen::MatrixXd centered = points.rowwise() - centroid.transpose();
        Eigen::Matrix3d cov = (centered.adjoint() * centered) / double(points.rows() - 1);
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
        Eigen::Vector3d axis = eig.eigenvectors().col(2);
        
        // 方向一致性检查
        Eigen::Vector3d recent_dir = (pose_window_.back().position - pose_window_.front().position).normalized();
        if (axis.dot(recent_dir) < 0) {
            axis = -axis;
        }
        
        // 如果没有直径信息，直接使用PCA结果
        if (pipe_diameter_ <= 0) {
            current_output_.local_axis = axis;
            current_output_.pipe_center = centroid;
            return;
        }
        
        // 使用直径约束优化中心线估计
        // 步骤1: 创建垂直于初始轴向的平面
        Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(centroid, axis);
        
        // 步骤2: 将点投影到平面并拟合圆心
        Eigen::Vector3d center_sum = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> projected_points;
        
        for (const auto& pose : pose_window_) {
            Eigen::Vector3d proj = plane.projection(pose.position);
            projected_points.push_back(proj);
            center_sum += proj;
        }
        
        // 初始圆心估计（投影点的质心）
        Eigen::Vector3d circle_center = center_sum / projected_points.size();
        
        // 步骤3: 优化圆心位置（最小化到圆心的距离与半径的差异）
        // 简单实现：使用质心作为圆心
        // 更高级的实现可以使用圆拟合算法（如最小二乘圆拟合）
        
        // 步骤4: 修正位置点（将它们移动到圆周上）
        std::vector<Eigen::Vector3d> corrected_points;
        for (size_t j = 0; j < projected_points.size(); j++) {
            Eigen::Vector3d radial_vec = projected_points[j] - circle_center;
            double distance = radial_vec.norm();
            
            if (distance > 1e-5) {  // 避免除以零
                // 将点修正到指定半径的圆周上
                Eigen::Vector3d corrected_proj = circle_center + (radial_vec / distance) * pipe_radius_;
                
                // 将修正后的投影点映射回原始空间
                // 计算原始点到投影平面的距离
                Eigen::Vector3d normal = plane.normal().normalized();
                double signed_distance = plane.signedDistance(pose_window_[j].position);
                
                // 重建修正后的3D点
                Eigen::Vector3d corrected_point = corrected_proj + normal * signed_distance;
                corrected_points.push_back(corrected_point);
            } else {
                // 如果点太接近圆心，保持原位置
                corrected_points.push_back(pose_window_[j].position);
            }
        }
        
        // 步骤5: 使用修正后的点重新计算轴向
        Eigen::MatrixXd corrected_mat(corrected_points.size(), 3);
        for (size_t j = 0; j < corrected_points.size(); j++) {
            corrected_mat.row(j) = corrected_points[j];
        }
        
        Eigen::Vector3d corrected_centroid = corrected_mat.colwise().mean();
        Eigen::MatrixXd centered_corrected = corrected_mat.rowwise() - corrected_centroid.transpose();
        Eigen::Matrix3d cov_corrected = (centered_corrected.adjoint() * centered_corrected) / double(corrected_points.size() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_corrected(cov_corrected);
        Eigen::Vector3d corrected_axis = eig_corrected.eigenvectors().col(2);
        
        // 更新输出
        current_output_.local_axis = corrected_axis;
        current_output_.pipe_center = circle_center;
    }

    // 计算向量在截面平面上的投影
    Eigen::Vector3d projectToSectionPlane(const Eigen::Vector3d& vec, 
                                         const Eigen::Vector3d& axis) const {
        return vec - vec.dot(axis) * axis;
    }

    // 计算周向角度（使用直径约束优化）
    double calculateCircumferentialAngle(const Pose& pose) {
        // 如果没有直径信息，使用重力参考
        if (pipe_diameter_ <= 0) {
            return calculateAngleWithGravity(pose);
        }
        
        // 使用径向方向参考
        return calculateAngleWithRadialReference(pose);
    }

    // 使用重力参考计算角度
    double calculateAngleWithGravity(const Pose& pose) {
        Eigen::Vector3d forward_body(1, 0, 0);
        Eigen::Vector3d forward_world = pose.orientation * forward_body;
        Eigen::Vector3d proj_forward = projectToSectionPlane(forward_world, current_output_.local_axis);
        
        if (proj_forward.norm() < 1e-5) {
            return current_output_.circumferential_angle;
        }
        proj_forward.normalize();
        
        if (ref_direction_.norm() < 1e-5) {
            Eigen::Vector3d gravity_current = pose.orientation * gravity_world_;
            ref_direction_ = projectToSectionPlane(gravity_current, current_output_.local_axis);
            ref_direction_.normalize();
        }
        
        Eigen::Vector3d cross = ref_direction_.cross(proj_forward);
        double sin_angle = cross.dot(current_output_.local_axis);
        double cos_angle = ref_direction_.dot(proj_forward);
        
        return std::atan2(sin_angle, cos_angle);
    }

    // 使用径向参考计算角度（更稳定）
    double calculateAngleWithRadialReference(const Pose& pose) {
        // 1. 计算从管道中心指向机器人位置的向量
        Eigen::Vector3d radial_vec = pose.position - current_output_.pipe_center;
        
        // 2. 投影到截面平面
        Eigen::Vector3d radial_proj = projectToSectionPlane(radial_vec, current_output_.local_axis);
        
        if (radial_proj.norm() < 1e-5) {
            // 机器人位于中心线上，角度未定义
            return current_output_.circumferential_angle;
        }
        radial_proj.normalize();
        
        // 3. 计算相对于初始径向方向的角度
        Eigen::Vector3d cross = initial_radial_dir_.cross(radial_proj);
        double sin_angle = cross.dot(current_output_.local_axis);
        double cos_angle = initial_radial_dir_.dot(radial_proj);
        
        return std::atan2(sin_angle, cos_angle);
    }

private:
    // 配置参数
    int window_size_;                  // 滑动窗口大小
    double pipe_diameter_ = 0.0;       // 管道直径
    double pipe_radius_ = 0.0;         // 管道半径
    Eigen::Vector3d gravity_world_;    // 世界坐标系中的重力方向
    
    // 状态变量
    bool is_initialized_ = false;      // 初始化标志
    std::deque<Pose> pose_window_;     // 位姿滑动窗口
    Pose last_pose_;                   // 上一个位姿
    double axial_displacement_ = 0.0;  // 累积轴向位移
    Eigen::Vector3d ref_direction_;    // 参考方向（重力投影）
    Eigen::Vector3d initial_radial_dir_; // 初始径向方向（从中心指向机器人）
    PipeOdometry current_output_;      // 当前输出
};

// 使用示例
int main() {
    // 初始化转换器（指定管道直径为0.5米）
    PipeOdometryConverter converter(10, 0.5, Eigen::Vector3d(0, 0, -1));
    
    // 模拟VINS-Fusion的位姿输入
    PipeOdometryConverter::Pose new_pose;
    new_pose.position = Eigen::Vector3d(0, 0.2, 0); // 偏离中心位置
    new_pose.orientation = Eigen::Quaterniond::Identity();
    new_pose.timestamp = 0.0;
    
    // 处理位姿
    auto pipe_odom = converter.processPose(new_pose);
    
    // 输出结果
    std::cout << "Pipe Center: (" << pipe_odom.pipe_center.x() << ", "
              << pipe_odom.pipe_center.y() << ", " << pipe_odom.pipe_center.z() << ")\n";
    std::cout << "Axial Displacement: " << pipe_odom.axial_displacement
              << ", Circumferential Angle: " << pipe_odom.circumferential_angle
              << std::endl;
    
    return 0;
}