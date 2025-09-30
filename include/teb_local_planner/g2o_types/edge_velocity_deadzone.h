
#ifndef EDGE_VELOCITY_DEADZONE_H_
#define EDGE_VELOCITY_DEADZONE_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include <geometry_msgs/Twist.h>




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
class EdgeVelocityDeadzone : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocityDeadzone()
  {
    this->resize(3);
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocityDeadzone()");

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
    
    
    // 2. 为 vx 设计“死区”惩罚函数
    double deadzone_radius = 0.1; // m/s, 可配置
    double vx_abs = std::abs(vx);
    double vx_error = 0.0;

    double max_error = cost_function(0.5*deadzone_radius, deadzone_radius); // 计算在死区边界一半处的最大惩罚值
    double min_error = 0.0;
    double error_range = max_error - min_error;

    if (vx_abs > 1e-6 && vx_abs < deadzone_radius) // 1e-6 to avoid penalty at perfect zero
    {
        vx_error = cost_function_1(vx_abs, deadzone_radius);
    }
    vx_error *= 100.0; // 归一化到 [0, 1]
    _error[0] = vx_error;

    // 3. 为 vy 设计“死区”惩罚函数 (完全相同)
    double vy_abs = std::abs(vy);
    double vy_error = 0.0;

    if (vy_abs > 1e-6 && vy_abs < deadzone_radius)
    {
        vy_error = cost_function_1(vy_abs, deadzone_radius);
    }
    vy_error *=100.0; // 归一化到 [0, 1]
    _error[1] = vy_error;

    ROS_INFO_THROTTLE(10, "EdgeVelocityDeadzone::computeError() vx=%f, vy=%f, vx_error=%f, vy_error=%f", vx, vy, _error[0], _error[1]);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocityDeadzone::computeError() wheel swerve angle: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeVelocityDeadzone::computeError() wheel swerve angle: _error[1]=%f\n",_error[1]);

  }

  double cost_function(double x, double pivot)
  {
      return -x*x + pivot*x; // 示例：一个简单的二次函数，形成一个“山谷”形状
  }

  double cost_function_1(double v, double deadzone){
    double half_width = deadzone / 2.0;
    double cost_smooth = std::exp((-v*v)/(2*half_width*half_width));

    double dip_width = 0.01;
    double dip_smooth = 1.0-std::exp((-v*v)/(2*dip_width*dip_width));

    double v_error_smooth = cost_smooth * dip_smooth;
    return v_error_smooth;
  }

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};

} // end namespace teb_local_planner

#endif // EDGE_KINEMATICS_SWERVE_H_