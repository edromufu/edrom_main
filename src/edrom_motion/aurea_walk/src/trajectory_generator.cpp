#include "aurea_walk/trajectory_generator.hpp"
#include <cmath>

namespace aurea_walk
{

// Interpolação suave (cosseno) de 1 a 0 durante a fase de apoio simples
double get_h_phi(double phi, double phi_b, double phi_e)
{
  if (phi < phi_b) {return 1.0;}
  if (phi >= phi_e) {return 0.0;}
  return 0.5 * (1.0 + std::cos(M_PI * (phi - phi_b) / (phi_e - phi_b)));
}

// Interpolação para levantar e abaixar o pé suavemente
double get_v_phi(double phi, double phi_b, double phi_e)
{
  if (phi < phi_b || phi >= phi_e) {return 0.0;}
  return 0.5 * (1.0 - std::cos(2.0 * M_PI * (phi - phi_b) / (phi_e - phi_b)));
}

void get_com_pose_at_time(
  Eigen::Vector2d & out_com_pos, double & out_com_yaw,
  double t, double T, double ds_ratio, double z_com, double g,
  const PoseData & p_start, const PoseData & p_end, 
  const PoseData & p_support)
{
  // --- Interpolação do Yaw (continua geométrica) ---
  double tb = (T * ds_ratio) / 2.0;
  double te = T - tb;
  double phi = t / T;
  double h_phi = get_h_phi(phi, tb / T, te / T);
  out_com_yaw = h_phi * p_start.yaw + (1.0 - h_phi) * p_end.yaw;

  // --- Cálculo da Trajetória Dinâmica (X e Y) ---

  if (z_com <= 0) {
    out_com_pos = p_start.position;
    return;
  }
  double lambda_val = std::sqrt(g / z_com);

  Eigen::Vector2d m_d1;
  if (tb > 1e-6) {
    m_d1 = (p_support.position - p_start.position) / tb;
  } else {
    m_d1 = Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d m_d2;
  if ((T - te) > 1e-6) {
    m_d2 = (p_end.position - p_support.position) / (T - te);
  } else {
    m_d2 = Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d k_d1 = (1.0 / lambda_val) * m_d1 * std::sinh(-lambda_val * tb);
  Eigen::Vector2d k_d2 = (1.0 / lambda_val) * m_d2 * std::sinh(lambda_val * (T - te));
  
  double e_lambda_T = std::exp(lambda_val * T);
  double e_neg_lambda_T = std::exp(-lambda_val * T);
  double denominator = e_lambda_T - e_neg_lambda_T;
  if (std::abs(denominator) < 1e-6) {
      out_com_pos = p_start.position;
      return;
  }
  Eigen::Vector2d c1 = (k_d2 - k_d1 * e_neg_lambda_T) / denominator;
  Eigen::Vector2d c2 = (k_d1 * e_lambda_T - k_d2) / denominator;

  if (t < tb) {
      // MODIFICADO: Usa interpolação suave (cosseno) em vez de linear
      double ds_normalized_time = t / tb;
      double h = 0.5 * (1.0 - cos(M_PI * ds_normalized_time));
      Eigen::Vector2d zmp_ref = (1.0 - h) * p_start.position + h * p_support.position;
      
      // ANTIGO: Eigen::Vector2d zmp_ref = p_start.position + m_d1 * t;

      Eigen::Vector2d sinh_term = (1.0 / lambda_val) * m_d1 * std::sinh(lambda_val * (t - tb));
      out_com_pos = zmp_ref + c1 * std::exp(lambda_val * t) + c2 * std::exp(-lambda_val * t) - sinh_term;
  
  } else if (t >= te) {
      // MODIFICADO: Usa interpolação suave (cosseno) em vez de linear
      double ds_normalized_time = (t - te) / (T - te);
      double h = 0.5 * (1.0 - cos(M_PI * ds_normalized_time));
      Eigen::Vector2d zmp_ref = (1.0 - h) * p_support.position + h * p_end.position;

      // ANTIGO: Eigen::Vector2d zmp_ref = p_support.position + m_d2 * (t - te);

      Eigen::Vector2d sinh_term = (1.0 / lambda_val) * m_d2 * std::sinh(lambda_val * (t - te));
      out_com_pos = zmp_ref + c1 * std::exp(lambda_val * t) + c2 * std::exp(-lambda_val * t) - sinh_term;
  
  } else {
      // Fase de apoio único, ZMP permanece no centro do pé de apoio
      Eigen::Vector2d zmp_ref = p_support.position;
      out_com_pos = zmp_ref + c1 * std::exp(lambda_val * t) + c2 * std::exp(-lambda_val * t);
  }
}

void get_swing_foot_pose_at_time(
  Eigen::Vector3d & out_swing_pos_world, double & out_swing_yaw_world,
  double t, double T, double z_step, double ds_ratio,
  const PoseData & p_start, const PoseData & p_end)
{
  double tb = (T * ds_ratio) / 2.0;
  double te = T - tb;
  double phi = t / T;
  double phi_b = tb / T;
  double phi_e = te / T;

  double h_phi = get_h_phi(phi, phi_b, phi_e);
  double v_phi = get_v_phi(phi, phi_b, phi_e);

  Eigen::Vector2d swing_pos_2d = h_phi * p_start.position + (1.0 - h_phi) * p_end.position;
  out_swing_yaw_world = h_phi * p_start.yaw + (1.0 - h_phi) * p_end.yaw;

  out_swing_pos_world.x() = swing_pos_2d.x();
  out_swing_pos_world.y() = swing_pos_2d.y();
  
  double landing_phase_start = phi_e * 0.85; // Começa a fase de pouso nos últimos 15%

  if (phi > landing_phase_start) {
    double phase_phi = (phi - landing_phase_start) / (phi_e - landing_phase_start);
    out_swing_pos_world.z() = (1.0 - phase_phi) * (z_step * get_v_phi(landing_phase_start * T, phi_b, phi_e));
  } else {
    out_swing_pos_world.z() = z_step * v_phi;
  }
}

void select_next_poses(
  PoseData & out_next_torso, PoseData & out_next_swing,
  const PoseData & current_torso, const PoseData & swing_foot, const geometry_msgs::msg::Twist & v_cmd,
  double T, double y_sep)
{
  double dx = v_cmd.linear.x * T;
  double dy = v_cmd.linear.y * T;
  double d_yaw = v_cmd.angular.z * T;

  Eigen::Rotation2Dd rot(current_torso.yaw);
  Eigen::Vector2d world_displacement = rot * Eigen::Vector2d(dx, dy);

  out_next_torso.position = current_torso.position + world_displacement;

  out_next_torso.yaw = current_torso.yaw + d_yaw;

  double foot_offset_y = swing_foot.is_left ? y_sep : -y_sep;
  
  Eigen::Rotation2Dd next_rot(out_next_torso.yaw);
  Eigen::Vector2d foot_offset_in_torso_frame(v_cmd.linear.x * T / 2.0, foot_offset_y);
  Eigen::Vector2d world_foot_offset = next_rot * foot_offset_in_torso_frame;
  
  out_next_swing.position = out_next_torso.position + world_foot_offset;
  out_next_swing.yaw = out_next_torso.yaw;
}



}  // namespace op3_kinematics