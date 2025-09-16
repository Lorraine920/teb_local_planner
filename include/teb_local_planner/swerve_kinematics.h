// in teb_local_planner/include/teb_local_planner/swerve_kinematics.h

#ifndef SWERVE_KINEMATICS_H_
#define SWERVE_KINEMATICS_H_

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <teb_local_planner/teb_config.h> // 假设轮子配置会来自TebConfig

namespace teb_local_planner
{

/**
 * @struct WheelState
 * @brief Represents the state of a single swerve wheel.
 *
 * This struct holds the calculated steering angle and drive speed for one wheel.
 */
struct WheelState
{
  double steering_angle = 0.0; //!< Steering angle of the wheel in radians.
  double drive_speed = 0.0;    //!< Drive speed of the wheel in m/s. Can be negative.
};


/**
 * @class SwerveKinematics
 * @brief Implements the inverse kinematics for a four-wheel swerve drive robot.
 *
 * This class calculates the required steering angles and drive speeds for each wheel
 * to achieve a desired robot velocity (vx, vy, omega).
 * The implementation is designed to be smooth and differentiable around vx=0 to be
 * compatible with gradient-based optimizers like g2o.
 */
class SwerveKinematics
{
public:

  /**
   * @brief Default constructor.
   */
  SwerveKinematics() = default;

  /**
   * @brief Constructor that initializes the model from a configuration.
   * @param cfg TebConfig containing robot parameters.
   */
  SwerveKinematics(const TebConfig& cfg)
  {
    setConfig(cfg);
  }

  /**
   * @brief Initializes or updates the kinematics model from a configuration.
   *
   * This method sets up the wheel positions based on the robot's wheelbase and track width.
   * @param cfg TebConfig containing robot parameters like wheelbase and track_width.
   */
  void setConfig(const TebConfig& cfg)
  {
    // Let's assume wheel positions are defined by wheelbase (L) and track width (W)
    // You might get these from cfg.robot.wheelbase and cfg.robot.track_width
    // You'll need to add these parameters to your TebConfig struct.
    double L = cfg.robot.wheelbase; // Distance between front and rear axles
    double W = cfg.robot.track_width; // Distance between left and right wheels

    wheel_positions_.resize(4);
    // Wheel 0: Front Right
    wheel_positions_[0] = Eigen::Vector2d(L / 2.0, -W / 2.0);
    // Wheel 1: Front Left
    wheel_positions_[1] = Eigen::Vector2d(L / 2.0, W / 2.0);
    // Wheel 2: Rear Left
    wheel_positions_[2] = Eigen::Vector2d(-L / 2.0, W / 2.0);
    // Wheel 3: Rear Right
    wheel_positions_[3] = Eigen::Vector2d(-L / 2.0, -W / 2.0);
  }

  /**
   * @brief Calculates the inverse kinematics for the swerve drive.
   *
   * Given a desired robot velocity, this function computes the state (steering angle and
   * drive speed) for each of the four wheels. The calculation is continuous and avoids
   * angle flips for small changes in velocity, especially around vx=0.
   *
   * @param vx Desired longitudinal velocity in the robot's frame (m/s).
   * @param vy Desired lateral (strafing) velocity in the robot's frame (m/s).
   * @param omega Desired angular velocity of the robot (rad/s).
   * @return A vector of WheelState structs, one for each wheel.
   */
  std::vector<WheelState> inverseKinematics(double vx, double vy, double omega) const
  {
    std::vector<WheelState> wheel_states(4);

    for (int i = 0; i < 4; ++i)
    {
      // Calculate the velocity vector for this wheel due to robot rotation
      // This is omega x r, where r is the position vector of the wheel
      double rot_vx = -omega * wheel_positions_[i].y();
      double rot_vy =  omega * wheel_positions_[i].x();

      // Total velocity vector for this wheel
      double total_vx = vx + rot_vx;
      double total_vy = vy + rot_vy;

      // Calculate desired drive speed and steering angle
      wheel_states[i].drive_speed = std::sqrt(total_vx * total_vx + total_vy * total_vy);
      wheel_states[i].steering_angle = std::atan2(total_vy, total_vx);
    }
    
    return wheel_states;
  }

  /**
   * @brief An alternative inverse kinematics implementation that allows wheel reversal.
   *
   * This version is often preferred for real robots as it can prevent large
   * steering angle changes by reversing the wheel direction instead.
   * You would need to pass the previous steering angles to make an optimal choice.
   * For the g2o edge, the simpler `inverseKinematics` is often a better start.
   *
   * @param vx Desired longitudinal velocity in the robot's frame (m/s).
   * @param vy Desired lateral (strafing) velocity in the robot's frame (m/s).
   * @param omega Desired angular velocity of the robot (rad/s).
   * @param prev_steering_angles The steering angles from the previous time step.
   * @return A vector of WheelState structs, one for each wheel.
   */
  std::vector<WheelState> inverseKinematicsReal(double vx, double vy, double omega, const std::vector<double>& prev_steering_angles) const
  {
    std::vector<WheelState> wheel_states(4);

    for (int i = 0; i < 4; ++i)
    {
      // Calculate the velocity vector for this wheel due to robot rotation
      // This is omega x r, where r is the position vector of the wheel
      double rot_vx = -omega * wheel_positions_[i].y();
      double rot_vy =  omega * wheel_positions_[i].x();

      // Total velocity vector for this wheel
      double total_vx = vx + rot_vx;
      double total_vy = vy + rot_vy;

      // Calculate desired drive speed and steering angle
      wheel_states[i].drive_speed = std::sqrt(total_vx * total_vx + total_vy * total_vy);
      wheel_states[i].steering_angle = std::atan2(total_vy, total_vx);

      // Optional: Normalize steering angle to [-pi/2, pi/2] and adjust drive speed sign
      if (wheel_states[i].steering_angle > M_PI / 2)
      {
        wheel_states[i].steering_angle -= M_PI;
        wheel_states[i].drive_speed = -wheel_states[i].drive_speed;
      }
      else if (wheel_states[i].steering_angle < -M_PI / 2)
      {
        wheel_states[i].steering_angle += M_PI;
        wheel_states[i].drive_speed = -wheel_states[i].drive_speed;
      }
    }
    
    return wheel_states;
  }


private:
  // A vector storing the (x, y) position of each wheel relative to the robot's center.
  std::vector<Eigen::Vector2d> wheel_positions_;

};

} // namespace teb_local_planner

#endif // SWERVE_KINEMATICS_H_