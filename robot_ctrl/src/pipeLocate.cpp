#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
        double timestamp;
    };

    /**
     * 构造函数
     * @param window_size 滑动窗口大小 (用于轴向估计的位姿数量)
     * @param gravity_dir 重力方向在世界坐标系中的向量 (通常为[0,0,-1]或[0,0,1])
     */
    PipeOdometryConverter(int window_size = 10, 
                          const Eigen::Vector3d& gravity_dir = Eigen::Vector3d(0, 0, -1))
        : window_size_(window_size), gravity_world_(gravity_dir.normalized()) {
        
        if (window_size < 3) {
            window_size_ = 3; // 最小窗口大小
        }
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

        // 1. 估计局部轴向
        current_output_.local_axis = estimateLocalAxis();

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

private:
    // 初始化参考系
    void initialize(const Pose& init_pose) {
        // 初始位置和姿态
        last_pose_ = init_pose;
        
        // 初始轴向估计 (使用初始运动方向或等待几个位姿)
        if (pose_window_.size() >= 2) {
            Eigen::Vector3d init_dir = (pose_window_.back().position - pose_window_.front().position).normalized();
            current_output_.local_axis = init_dir;
        } else {
            // 使用初始姿态的前向方向
            Eigen::Vector3d forward_vec(1, 0, 0); // 假设机器人前向是X轴
            current_output_.local_axis = init_pose.orientation * forward_vec;
        }
        
        // 计算初始参考方向
        Eigen::Vector3d gravity_current = init_pose.orientation * gravity_world_;
        ref_direction_ = projectToSectionPlane(gravity_current, current_output_.local_axis);
        ref_direction_.normalize();
        
        // 初始值设置
        axial_displacement_ = 0.0;
        current_output_.circumferential_angle = 0.0;
        current_output_.axial_displacement = 0.0;
        current_output_.timestamp = init_pose.timestamp;
        
        is_initialized_ = true;
    }

    // 使用PCA估计局部轴向
    Eigen::Vector3d estimateLocalAxis() {
        // 方法1：PCA
        Eigen::MatrixXd points(pose_window_.size(), 3);
        int i = 0;
        for (const auto& pose : pose_window_) {
            points.row(i++) = pose.position.transpose();
        }
        
        Eigen::Vector3d centroid = points.colwise().mean();
        Eigen::MatrixXd centered = points.rowwise() - centroid.transpose();
        Eigen::Matrix3d cov = (centered.adjoint() * centered) / double(points.rows() - 1);
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
        Eigen::Vector3d axis = eig.eigenvectors().col(2); // 最大特征值对应的特征向量
        
        // 检查方向是否与最近的运动一致
        Eigen::Vector3d recent_dir = (pose_window_.back().position - pose_window_.front().position).normalized();
        if (axis.dot(recent_dir) < 0) {
            axis = -axis; // 反转方向
        }
        
        return axis;
    }

    // 计算向量在截面平面上的投影
    Eigen::Vector3d projectToSectionPlane(const Eigen::Vector3d& vec, 
                                         const Eigen::Vector3d& axis) const {
        return vec - vec.dot(axis) * axis;
    }

    // 计算周向角度
    double calculateCircumferentialAngle(const Pose& pose) {
        // 获取机器人当前前进方向 (假设机器人本体系X轴为前进方向)
        Eigen::Vector3d forward_body(1, 0, 0);
        Eigen::Vector3d forward_world = pose.orientation * forward_body;
        
        // 投影到截面平面
        Eigen::Vector3d proj_forward = projectToSectionPlane(forward_world, current_output_.local_axis);
        if (proj_forward.norm() < 1e-5) {
            // 投影太小，方向不可靠，返回上一个角度
            return current_output_.circumferential_angle;
        }
        proj_forward.normalize();
        
        // 检查参考方向是否有效
        if (ref_direction_.norm() < 1e-5) {
            // 重新计算参考方向 (重力投影)
            Eigen::Vector3d gravity_current = pose.orientation * gravity_world_;
            ref_direction_ = projectToSectionPlane(gravity_current, current_output_.local_axis);
            ref_direction_.normalize();
        }
        
        // 计算角度 (使用叉积和点积)
        Eigen::Vector3d cross = ref_direction_.cross(proj_forward);
        double sin_angle = cross.dot(current_output_.local_axis); // 轴向分量决定符号
        double cos_angle = ref_direction_.dot(proj_forward);
        
        return std::atan2(sin_angle, cos_angle);
    }

private:
    // 配置参数
    int window_size_;                  // 滑动窗口大小
    Eigen::Vector3d gravity_world_;    // 世界坐标系中的重力方向
    
    // 状态变量
    bool is_initialized_ = false;      // 初始化标志
    std::deque<Pose> pose_window_;     // 位姿滑动窗口
    Pose last_pose_;                   // 上一个位姿
    double axial_displacement_ = 0.0;  // 累积轴向位移
    Eigen::Vector3d ref_direction_;    // 参考方向 (截面平面内)
    PipeOdometry current_output_;      // 当前输出
};

// 使用示例
int main() {
    // 初始化转换器 (滑动窗口=10，重力方向=[0,0,-1])
    PipeOdometryConverter converter(10, Eigen::Vector3d(0, 0, -1));
    
    // 模拟VINS-Fusion的位姿输入
    PipeOdometryConverter::Pose new_pose;
    new_pose.position = Eigen::Vector3d(0, 0, 0);
    new_pose.orientation = Eigen::Quaterniond::Identity();
    new_pose.timestamp = 0.0;
    
    // 处理位姿
    auto pipe_odom = converter.processPose(new_pose);
    
    // 输出: 轴向位移和周向角度
    std::cout << "Axial Displacement: " << pipe_odom.axial_displacement
              << ", Circumferential Angle: " << pipe_odom.circumferential_angle
              << std::endl;
    
    return 0;
}