#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <ceres/ceres.h>

class RobotMotionSolver {
public:
    struct Parameters {
        double l1, l2, l3, l4;
        double deg_alpha;
        double rw;
        double R, r;
        double target_v;
    };

    RobotMotionSolver(const Parameters& params) : params_(params) {}

    // 主求解函数 (输入 theta 角度，输出 beta 角度和轮速)
    bool Solve(double theta_deg, double* beta_rad, double* wheel_speed_axial, double* wheel_speed_circ) {
        const double theta_rad = theta_deg * M_PI / 180.0;
        
        // 1. 求解 beta
        if (!SolveBeta(theta_rad, beta_rad)) {
            return false;
        }
        // 2. 计算速度
        ComputeWheelSpeeds(theta_rad, *beta_rad, wheel_speed_axial, wheel_speed_circ);
        return true;
    }

private:
    // Ceres 代价函数定义
    struct BetaCostFunction {
        BetaCostFunction(double theta_rad, const Parameters& params):theta_rad_(theta_rad), params_(params) {}

        template <typename T>
        bool operator()(const T* beta, T* residual) const {
            const T input_value = T(params_.l3) + 
                                 (T(1.0) - ceres::cos(theta_rad_)) * 
                                 T(params_.r + params_.R) - 
                                 ceres::cos(theta_rad_) * T(params_.rw);

            const T term1 = (T(params_.l3) - T(params_.rw)) * 
                           (T(2.0) * ceres::cos(*beta) * ceres::cos(*beta) - T(1.0));
            
            const T term2 = T(2.0) * T(params_.l1) * ceres::cos(*beta) * 
                           ceres::sqrt(T(1.0) - ceres::cos(*beta) * ceres::cos(*beta));
            
            const T term3 = T(params_.l2) * 
                           ceres::sqrt(T(1.0) - ceres::cos(*beta) * ceres::cos(*beta));

            residual[0] = term1 + term2 + term3 - input_value;
            return true;
        }

        double theta_rad_;
        Parameters params_;
    };

    bool SolveBeta(double theta_rad, double* beta_rad) {
        double beta_init = 0.0;  // 初始猜测
        
        ceres::Problem problem;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<BetaCostFunction, 1, 1>(new BetaCostFunction(theta_rad, params_)), // cost function 
            nullptr,    // 使用默认的损失函数
            beta_rad    
        );

        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        return summary.IsSolutionUsable();
    }

    void ComputeWheelSpeeds(double theta_rad, double beta_rad,
                           double* v_axial, double* v_circ) {
        // 计算 theta_dot (MATLAB 中的 theta_dot = target_v / (R + r))
        const double theta_dot = params_.target_v / (params_.R + params_.r);

        // 使用 Ceres 自动微分计算 dbeta/dtheta
        double dbeta_dtheta = 0.0;
        {
            ceres::NumericDiffOptions options;
            std::unique_ptr<ceres::CostFunction> cost_function(
                new ceres::NumericDiffCostFunction<BetaThetaRatio, 
                    ceres::CENTRAL, 1, 1, 1>(
                    new BetaThetaRatio(params_)));
                
            double residuals;
            double parameters[2] = {beta_rad, theta_rad};
            double jacobians[2];
            cost_function->Evaluate(parameters, &residuals, jacobians);
            dbeta_dtheta = -jacobians[1] / jacobians[0];
        }

        // 计算 beta_dot
        const double beta_dot = dbeta_dtheta * theta_dot;

        // 计算 Q 点速度 (简化为 MATLAB 中的 x_q/y_q 导数)
        // ... (类似方法计算 dxq/dbeta, dyq/dbeta 等)

        // 转换为轮速 (示例)
        *v_axial = /* 根据你的物理模型计算 */;
        *v_circ = /* 根据你的物理模型计算 */;
    }

    Parameters params_;
};
