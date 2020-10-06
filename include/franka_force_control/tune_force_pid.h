// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <time.h>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>    

#include <franka_force_control/desired_mass_paramConfig.h>
#include <franka_hw/franka_cartesian_command_interface.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"


#include <franka_msgs/ErrorRecoveryAction.h>
#include <franka_msgs/ErrorRecoveryActionGoal.h>

#include <franka_force_control/median_filter.h>
#include <franka_force_control/pid.h>

#include "pulse.h"

#define MAX_FILTER_SAMPLES_NUM 100

namespace franka_force_control {

class TuneForcePidController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface,
                                   franka_hw::FrankaPoseCartesianInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  ros::Publisher tau_ext_pub, tau_d_pub, tau_cmd_pub, force_ext_pub, 
          force_g_pub, force_c_pub, force_nof_pub, force_ref_pub,
          force_cor_pub;
  ros::Publisher pos_ref_glob_pub, pos_glob_pub;
  ros::Publisher marker_pos_, marker_pip_;
  ros::Subscriber reset_sub_, force_torque_ref_, impedance_pos_ref_, gripper_type_sub_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Publisher ori_pid_roll_msg_pub_, ori_pid_pitch_msg_pub_, ori_pid_yaw_msg_pub_;
  ros::Publisher wrench_pid_pub_[6];

  ros::Publisher state_handle_pub_, model_handle_pub_;

  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  double desired_mass_{0.0};
  
  double desired_x_{0.0};
  double desired_y_{0.0};
  double desired_z_{0.0};
  double desired_tx_{0.0};
  double desired_ty_{0.0};
  double desired_tz_{0.0};
  
  double target_mass_{0.0};
  
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_tx_{3.14};
  double target_ty_{0.0};
  double target_tz_{0.0};
  
  double k_p_{0.0};
  double k_i_{0.0};

  bool starting_ = true;

  double target_k_p_{0.0};
  double target_k_i_{0.0};

  Eigen::Matrix<double, 6, 3> ft_pid;  
  Eigen::Matrix<double, 6, 3> ft_pid_target;  

  Eigen::Matrix<double, 1, 3> pid_ori_x_target, pid_ori_x;
  Eigen::Matrix<double, 1, 3> pid_ori_y_target, pid_ori_y;
  Eigen::Matrix<double, 1, 3> pid_ori_z_target, pid_ori_z;

  double f_p_x_{1.0};
  double f_i_x_{0.2};
  double f_d_x_{0.0};
  double f_p_y_{1.0};
  double f_i_y_{0.2};
  double f_d_y_{0.0};
  double f_p_z_{1.0};
  double f_i_z_{0.2};
  double f_d_z_{0.0};

  double t_p_x_{0.3};
  double t_i_x_{0.0};
  double t_d_x_{0.0};
  double t_p_y_{0.3};
  double t_i_y_{0.02};
  double t_d_y_{0.0};
  double t_p_z_{0.4};
  double t_i_z_{0.0};
  double t_d_z_{0.0};


  double integrator_limit_{10.0};

  Eigen::Matrix<double, 6, 1> f_err_prev_;
  Eigen::Matrix<double, 6, 1> f_err_int_;

  double filter_gain_{0.001};
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 6, 1> ext_force_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  static constexpr double kDeltaTauMax{1.0};

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_force_control::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_force_control::desired_mass_paramConfig& config,
                                uint32_t level);

  void reset_callback(const franka_msgs::ErrorRecoveryActionGoal&  msg);
  void ft_ref_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);


  void getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix, float *angles);

  void filter_new_params();
  void wrapErrToPi(float* orientation_meas);

  void debug_publish_tau(Eigen::VectorXd tau_ext, Eigen::VectorXd tau_d, Eigen::VectorXd tau_cmd_pid);



  median_filter filter_x_, filter_y_, filter_z_;

  median_filter force_filter_[6];

  float force_filt_x_, force_filt_y_, force_filt_z_;
  std::array<double, 6> force_filtered_;

  Eigen::Matrix<double, 6, 1> force_meas_init_;

  int median_size_;

  // impedance
  double pos_step_{0.01}, time_step_{0.0};
  double loc_d_x_{0.0}, loc_d_y_{0.0}, loc_d_z_{0.0};
  double count_time_{0.0};
  Eigen::Matrix<double, 3, 1> pos_global_prev_, vel_global_prev_;
  Eigen::Matrix<double, 4, 1> dXglobal_;
  double imp_f_{0.1}, imp_m_{3.}, imp_d_{31.6228}, imp_k_{50.}, imp_scale_{5.0}, imp_scale_m_{0.0}, imp_scale_m_i_{0.0};
  double target_imp_f_{0.}, target_imp_m_{0.}, target_imp_d_{10.}, target_imp_k_{2.5}, target_imp_scale_{4.0}, target_imp_scale_m_{0.0}, target_imp_scale_m_i_{0.0};

  Eigen::Matrix<double, 4, 1> posGlEE_prev_;

  int wiggle_timer_{0};
  Eigen::Matrix<double, 3, 1> wiggle_moments_;
  float vel_ampl_prev_{0.};

  PID orientation_pid_roll, orientation_pid_pitch, orientation_pid_yaw;
  PID wrench_pid[6];

  ros::Duration elapsed_time_{0};
  sequence seq;


  bool prev_active_ = true;
  float dir_ref_ = 1;
  float ref = 0;
  // float ref_max = 0.1;
  // float dref = 0.01;
  float ref_max = 0.0;
  float dref = 0.00;

  int ctrld = 0;




  void ramp_ref();

  int ramp_phase{0}, ramp_cnt{0};
  int joint_torque_ctrl_id;
  std::vector<int> step_count;
  std::vector<float> ramp_slopes;
  std::vector<float> steps;
  float joint_torque_ref_{0.0};
  bool ramp_first_pass{true};
};

}  // namespace franka_force_control
