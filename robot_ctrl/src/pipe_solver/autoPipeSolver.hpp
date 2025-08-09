#include <ceres/ceres.h>      
#include <Eigen/Dense>
#include <cmath>
#include "robot_params.hpp"
#include "Timer.hpp"

// todo 用于定义求解器计算过程中的状态
enum SolverState
{
    Normal = 0,              // 正常状态
    BetaSolveFailed,         // beta求解失败
    WheelSpeedComputeFailed, // 轮速计算失败
    solveEnd                 // 求解结束
};

// * 用于构建非线性求解函数的数学参数模型，后续在求解器内部使用
struct MathParameters{
    // 机器人几何尺寸
    double l1 = 62.0;
    double l2 = 270.0;
    double l3 = 115.0;
    double deg_alpha = 51.8748;
    double rw = 50.0;
    
    // 定义默认参数，支持外部修改
    double l4 = 280.12;
    double R = 300.0; // 弯管转弯半径
    double r = 180.0; // 弯管管道半径
};

// todo 配置非线性函数
// * 核心的描述beta与theta角关系的函数 对应matlab中的函数 f = @(b , t)
struct BETA_THETA_FITTING_FUNC{
    BETA_THETA_FITTING_FUNC( const MathParameters& params): params_(params) { };
    // 补充定义类内部成员
    MathParameters params_;

    // 定义基于模板的函数体，对应matlab中的f函数
    template <typename T>
    bool operator()(const T* b , const T* t, T* residual) const{
        // l3 + (1 - cos(t)) * (r + R) - cos(t) * rw 
        const T input_value =   T(params_.l3) + 
                                T( 1.0 - ceres::cos(*t) ) *
                                T(params_.R + params_.r) -
                                ceres::cos(*t) * T(params_.rw);
        // (l3 - rw) *(2 * cos(b) * cos(b) - 1) 
        const T term1 = (T(params_.l3) - T(params_.rw)) *
                        (T(2.0) * ceres::cos(*b) * ceres::cos(*b) - T(1.0));
        // 2 * l1 * cos(b) * sqrt( 1 - cos(b) * cos(b))
        const T term2 = T(2.0) * T(params_.l1) * ceres::cos(*b) *
                        ceres::sqrt(T(1.0) - ceres::cos(*b) * ceres::cos(*b));
        // l2 * sqrt( 1 - cos(b)*cos(b))
        const T term3 = T(params_.l2) * 
                        ceres::sqrt(T(1.0) - ceres::cos(*b) * ceres::cos(*b));
        //  f = term1 + term2 + term3 - input_value
        residual[0] = term1 + term2 + term3 - input_value;
        return true;
    }
};

// * 专门用于Ceres优化的单参数版本，theta作为固定值
struct BETA_THETA_FITTING_FUNC_SINGLE_PARAM{
    BETA_THETA_FITTING_FUNC_SINGLE_PARAM(const MathParameters& params, double fixed_theta)
        : params_(params), theta_(fixed_theta) { };
    
    MathParameters params_;
    double theta_;

    // 只优化beta，theta为固定值
    template <typename T>
    bool operator()(const T* b, T* residual) const{
        // l3 + (1 - cos(t)) * (r + R) - cos(t) * rw 
        const T input_value =   T(params_.l3) + 
                                T( 1.0 - ceres::cos(T(theta_)) ) *
                                T(params_.R + params_.r) -
                                ceres::cos(T(theta_)) * T(params_.rw);
        // (l3 - rw) *(2 * cos(b) * cos(b) - 1) 
        const T term1 = (T(params_.l3) - T(params_.rw)) *
                        (T(2.0) * ceres::cos(*b) * ceres::cos(*b) - T(1.0));
        // 2 * l1 * cos(b) * sqrt( 1 - cos(b) * cos(b))
        const T term2 = T(2.0) * T(params_.l1) * ceres::cos(*b) *
                        ceres::sqrt(T(1.0) - ceres::cos(*b) * ceres::cos(*b));
        // l2 * sqrt( 1 - cos(b)*cos(b))
        const T term3 = T(params_.l2) * 
                        ceres::sqrt(T(1.0) - ceres::cos(*b) * ceres::cos(*b));
        //  f = term1 + term2 + term3 - input_value
        residual[0] = ceres::pow(term1 + term2 + term3 - input_value , 2);
        return true;
    }
};

struct X_Q_FUNC{
    X_Q_FUNC( const MathParameters& params)
        : params_(params) {  };
    // 补充定义类内部成员
    MathParameters params_;
    

    // 定义基于模板的函数体，对应matlab中的 x_q 函数
    template <typename T>
    bool operator()(const T*b , const T* t , T* residual) const{
        // x =  (l1 + l2 / (2 * cos(b) )) * (1 + cos(2 * b)) - 
        //      (l3 - rw) * sin(2 * b) - 
        //      sin(t) * (R + r + rw);
        const T term_x1 = (T(params_.l1) + T(params_.l2) / (T(2.0) * ceres::cos(*b))) * 
                            (T(1.0) + ceres::cos(T(2.0) * (*b)));
        const T term_x2 = (T(params_.l3 - params_.rw)) * 
                            ceres::sin(T(2.0) * (*b));
        const T term_x3 = ceres::sin(*t) * T(params_.R + params_.r + params_.rw);
        const T x = term_x1 - term_x2 - term_x3;

        // t_ac = l1 + l2 / (2 * cos(b));
        const T t_ac = T(params_.l1) + T(params_.l2) / (T(2.0) * ceres::cos(*b));

        // x_q = t_ac(b) * (1 + cos( 2 * b)) - l4 * sin( 2 * b ) - x(b , t);
        residual[0] = t_ac * (T(1.0) + ceres::cos(T(2.0) * (*b))) - 
                      T(params_.l4) * ceres::sin(T(2.0) * (*b)) - x;
        return true;
    }
};

struct Y_Q_FUNC{
    Y_Q_FUNC( const MathParameters& params)
        : params_(params) {  };
    // 补充定义类内部成员
    MathParameters params_;

    // 定义基于模板的函数体，对应matlab中的 y_q 函数
    // * y_q = -t_ac(b) * sin(2 * b) - l4 * cos(2 *b) + l3 + R + r;
    template <typename T>
        bool operator()(const T*b  , T* residual) const{
        // t_ac = l1 + l2 / (2 * cos(b));
        const T t_ac = T(params_.l1) + T(params_.l2) / (T(2.0) * ceres::cos(*b));

        // y_q = -t_ac(b) * sin(2 * b) - l4 * cos(2 *b) + l3 + R + r;
        residual[0] =   -t_ac * ceres::sin(T(2.0 * (*b))) - 
                        T(params_.l4) * ceres::cos(T(2.0 * (*b))) + 
                        T(params_.l3) + T(params_.R) + T(params_.r);
        return true;
    }
};

class AutoPipeSolver{
public:
    AutoPipeSolver();
    ~AutoPipeSolver();

    // todo 用于求解的参数与变量
    MathParameters params_; // 用于存储数学模型参数的结构体
    double theta_deg_ = 0.0;
    double theta_rad_ = 0.0; // 输入的theta角度，单位为弧度
    double beta_rad_ = 0.0;  // 输出的beta角度，
    double beta_deg_ = 0.0;  // 输出的beta角度，单位为度   
    double target_v_ = 0.02; // 机器人的目标主动轮线速度
    double cur_dis_x_ = 900.0;
    
    // todo 外部接口
    void set_pipe_params(double l4 , double R , double r );
    SolverState solve(double theta_deg, double target_v ,      // 两个输入 
                float * push_length , float* main_wheel_speed , float* assist_wheel_speed); // 三个输出
        
private:
    MYTIMER timer_; // 定义计时器，用于性能分析

    // 定义需要用到的自动求导函数
    ceres::CostFunction* theta_beta_func_ = nullptr;        // 双输入参数的f函数， 用于求自动微分
    ceres::CostFunction* diff_func_beta_singel_ = nullptr;  // 单输入参数的f函数， 用于基于theta求beta
    ceres::CostFunction* xq_func_ = nullptr;           // 用于计算x_q的函数
    ceres::CostFunction* yq_func_ = nullptr;           // 用于计算y_q的函数
    


    // 用于生成自动求导函数
    void generate_diff_funcs_();
    
    SolverState solve_beta_();
    SolverState compute_wheel_speeds_(float* main_wheel_speed , float* assist_wheel_speed);
    
    double dis_x_func_(double theta_r , double beta_r);



};
