
#ifndef EDGE_KINEMATICS_SWERVE_H_
#define EDGE_KINEMATICS_SWERVE_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include <geometry_msgs/Twist.h>

#include "teb_local_planner/swerve_kinematics.h"



namespace teb_local_planner
{

/**
 * @class EdgeAcceleration
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot } ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeKinematicsSwerveStart() and EdgeKinematicsSwerveGoal() for defining boundary values!
 */    
class EdgeKinematicsSwerve : public BaseTebMultiEdge<4, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeKinematicsSwerve()
  {
    this->resize(3);
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeKinematicsSwerve()");
    
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    const Eigen::Vector2d diff = pose2->position() - pose1->position();

    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta());
    double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
    double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();
    double vx = p1_dx / dt->dt();
    double vy = p1_dy / dt->dt();
    double omega = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();

    SwerveKinematics sk(*cfg_);
    auto wheel_states = sk.inverseKinematics(vx, vy, omega);

    // ideal boundary[-pi/2, pi/2]
    double lower_bound = -M_PI_2;
    double upper_bound =  M_PI_2;

    for (int i = 0; i < wheel_states.size(); ++i)
    {
        double angle = wheel_states[i].steering_angle; // 范围 [-pi, pi]
        
        // =========================================================
        // 核心逻辑: 构建双谷代价函数
        // =========================================================

        // 定义两个稳定模式的中心点
        double mode1_center = 0.0;
        double mode2_center = M_PI;

        // 定义每个模式的“舒适区”宽度
        double sigma = 50 * M_PI / 180.0; // e.g., 45 degrees standard deviation

        // 计算当前角度到两个模式中心的“距离”
        // 注意要用 normalize_theta 来处理环绕
        double dist_to_mode1 = std::abs(std::abs(angle) - mode1_center);
        double dist_to_mode2 = std::abs(std::abs(angle) - mode2_center);

        // 使用高斯函数的倒数形式来创建“山谷”
        // exp(-x^2) 是一个钟形曲线，在x=0时为1，在x增大时趋于0。
        // 1 - exp(-x^2) 就是一个在x=0时为0，在x增大时趋于1的“山谷”形状。
        double valley1 = 1.0 - std::exp(- (dist_to_mode1 * dist_to_mode1) / (2 * sigma * sigma) );
        double valley2 = 1.0 - std::exp(- (dist_to_mode2 * dist_to_mode2) / (2 * sigma * sigma) );

        // 我们想要的是，当角度离任一中心点近时，成本都低。
        // 所以我们取这两个“山谷”函数的乘积。
        // 当 angle 接近 mode1 或 mode2 时，其中一个 valley 函数接近0，乘积就接近0。
        // 当 angle 处于两个模式中间时 (e.g., 90度)，两个 valley 函数都接近1，乘积也接近1 (成本最高)。
        double penalty = valley1 * valley2;

        _error[i] = penalty;
    }
    

    
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeKinematicsSwerve::computeError() wheel swerve angle: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeKinematicsSwerve::computeError() wheel swerve angle: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeKinematicsSwerve::computeError() wheel swerve angle: _error[2]=%f\n",_error[2]);
    ROS_ASSERT_MSG(std::isfinite(_error[3]), "EdgeKinematicsSwerve::computeError() wheel swerve angle: _error[3]=%f\n",_error[3]);
  }


      
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};

} // end namespace teb_local_planner

#endif // EDGE_KINEMATICS_SWERVE_H_