#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace aurea_walk
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
  const double L_COXA = 0.12;
  const double L_TIBIA = 0.085;
  const double D_QUADRIL_OFFSET = 0.0425;
  const double D_TORNOZELO_SOLA = 0.038;
};

} // namespace op3_kinematics