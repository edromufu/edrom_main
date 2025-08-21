#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace op3_kinematics
{

class IKSolver
{
public:
  IKSolver();
  bool solve(
    const Eigen::Vector3d & p_body, const Eigen::Quaterniond & R_body,
    const Eigen::Vector3d & p_foot_target, const Eigen::Quaterniond & R_foot_target,
    const std::string & leg_id,
    std::vector<double> & joint_angles);

private:
  const double L_COXA = 0.11;
  const double L_TIBIA = 0.08;
  const double D_QUADRIL_OFFSET = 0.05;
  const double D_TORNOZELO_SOLA = 0.025;
};

} // namespace op3_kinematics