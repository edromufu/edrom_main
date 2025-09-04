#include "aurea_walk/ik_solver.hpp" 
#include <cmath>
#include <iostream>

namespace aurea_walk
{

IKSolver::IKSolver() {}

bool IKSolver::solve(
  const Eigen::Vector3d & p_body, const Eigen::Quaterniond & R_body,
  const Eigen::Vector3d & p_foot_target, const Eigen::Quaterniond & R_foot_target,
  const std::string & leg_id,
  std::vector<double> & joint_angles)
{
  Eigen::Matrix3d R1 = R_body.toRotationMatrix();
  Eigen::Matrix3d R7 = R_foot_target.toRotationMatrix();

  Eigen::Vector3d p7_sola = p_foot_target;
  Eigen::Vector3d offset_sola = {0, 0, -D_TORNOZELO_SOLA};
  Eigen::Vector3d p7 = p7_sola - (R7 * offset_sola);

  try {
    double sinal_offset = (leg_id == "direita") ? -1.0 : 1.0;
    Eigen::Vector3d vetor_offset_quadril = {0, D_QUADRIL_OFFSET * sinal_offset, 0};
    Eigen::Vector3d p2 = p_body + (R1 * vetor_offset_quadril);

    Eigen::Vector3d r_vec = R7.transpose() * (p2 - p7);
    double rx = r_vec.x(), ry = r_vec.y(), rz = r_vec.z();

    double C = r_vec.norm();
    if (C > L_COXA + L_TIBIA) { return false; }

    double cos_q5 = (C * C - L_COXA * L_COXA - L_TIBIA * L_TIBIA) / (2 * L_COXA * L_TIBIA);
    cos_q5 = std::max(-1.0, std::min(1.0, cos_q5));
    double q5 = acos(cos_q5);

    double q7 = atan2(ry, rz);

    double sin_alpha = (L_COXA / C) * sin(q5);
    sin_alpha = std::max(-1.0, std::min(1.0, sin_alpha));
    double alpha = asin(sin_alpha);
    double q6 = -atan2(rx, std::copysign(1.0, rz) * sqrt(ry*ry + rz*rz)) - alpha;

    Eigen::Matrix3d R_q7_inv = Eigen::AngleAxisd(-q7, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d R_q56_inv = Eigen::AngleAxisd(-q5 - q6, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Matrix3d R_target = R1.transpose() * R7 * R_q7_inv * R_q56_inv;

    double R12 = R_target(0, 1), R22 = R_target(1, 1);
    double R31 = R_target(2, 0), R32 = R_target(2, 1), R33 = R_target(2, 2);

    double q2 = atan2(-R12, R22);
    double s2 = sin(q2), c2 = cos(q2);
    double q3 = atan2(R32, -R12 * s2 + R22 * c2);
    double q4 = atan2(-R31, R33);

    joint_angles = {q2, q3, q4, q5, q6, q7};
    return true;

  } catch (const std::exception & e) {
    std::cerr << "Erro na IK: " << e.what() << '\n';
    return false;
  }
}

} // namespace op3_ik_solver