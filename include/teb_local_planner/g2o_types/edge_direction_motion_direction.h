
#ifndef EDGE_DIRECTION_MOTION_DIRECTION_H_
#define EDGE_DIRECTION_MOTION_DIRECTION_H_

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
 */    
class EdgeDirectionMotion : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeDirectionMotion()
  {
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeKinematicsSwerve()");
    
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    const Eigen::Vector2d diff_1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff_2 = pose3->position() - pose2->position();

    double dot_product = diff_1.dot(diff_2);

    double penalty = 0.0;
    if(dot_product < 0){
      penalty = dot_product * dot_product;
    }
      _error[0] = penalty;

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDirectionMotion: _error[0]=%f\n",_error[0]);

  }

      
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};

} // end namespace teb_local_planner

#endif // EDGE_DIRECTION_MOTION_DIRECTION_H_