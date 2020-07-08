// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_force_control/tune_force_pid.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <franka/errors.h>

#include "pseudo_inversion.h"

#include "pulse.h"

#define LO -1.0
#define HI 1.0
#define N_ORDER 3
#define MIN_DIST_ 0.1
#define MAX_HIST_LEN_ 15
#define MIN_DIST_THRES_ 0.01

namespace franka_force_control {

bool TuneForcePid::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "TuneForcePid: Assuming SOFT gripper!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("TuneForcePid: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "TuneForcePid: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("TuneForcePid: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TuneForcePid: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("TuneForcePid: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TuneForcePid: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("TuneForcePid: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("TuneForcePid: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


  std::vector<double> fp, fi, fd, mp, mi, md;
  node_handle.getParam("/fp", fp);
  node_handle.getParam("/fi", fi);
  node_handle.getParam("/fd", fd);
  node_handle.getParam("/mp", mp);
  node_handle.getParam("/mi", mi);
  node_handle.getParam("/md", md);
  
  for (int i = 0; i < fp.size(); i++ ){
    ft_pid_target(i,0) = fp[i];
    ft_pid_target(i,1) = fi[i];
    ft_pid_target(i,2) = fd[i];
    ft_pid_target(i+3,0) = mp[i];
    ft_pid_target(i+3,1) = mi[i];
    ft_pid_target(i+3,2) = md[i];
  }

  std::vector<double> ori_x, ori_y, ori_z;
  node_handle.getParam("/ori_x", ori_x);
  node_handle.getParam("/ori_y", ori_y);
  node_handle.getParam("/ori_z", ori_z);
  
  for (int i = 0; i < 3; i++ ){
    pid_ori_x(0, i) = ori_x[i];
    pid_ori_y(0, i) = ori_y[i];
    pid_ori_z(0, i) = ori_z[i];
  }

  double limit = 10;
  orientation_pid_roll.set_kp(pid_ori_x(0, 0));
  orientation_pid_roll.set_ki(pid_ori_x(0, 1));
  orientation_pid_roll.set_kd(pid_ori_x(0, 2));
  orientation_pid_roll.set_lim_high(limit);
  orientation_pid_roll.set_lim_low(-limit);

  orientation_pid_pitch.set_kp(pid_ori_y(0, 0));
  orientation_pid_pitch.set_ki(pid_ori_y(0, 1));
  orientation_pid_pitch.set_kd(pid_ori_y(0, 2));
  orientation_pid_pitch.set_lim_high(limit);
  orientation_pid_pitch.set_lim_low(-limit);

  orientation_pid_yaw.set_kp(pid_ori_z(0, 0));
  orientation_pid_yaw.set_ki(pid_ori_z(0, 1));
  orientation_pid_yaw.set_kd(pid_ori_z(0, 2));
  orientation_pid_yaw.set_lim_high(limit);
  orientation_pid_yaw.set_lim_low(-limit);

  for (int i = 0; i < 6; i++ ){
    wrench_pid[i].set_kp(ft_pid_target(i,0));
    wrench_pid[i].set_ki(ft_pid_target(i,1));
    wrench_pid[i].set_kd(ft_pid_target(i,2));
    wrench_pid[i].set_lim_high(integrator_limit_);
    wrench_pid[i].set_lim_low(-integrator_limit_);
  }



  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_force_control::desired_mass_paramConfig>>(
      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&TuneForcePid::desiredMassParamCallback, this, _1, _2));

  tau_ext_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_ext", 1);
  tau_d_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_d", 1);
  tau_cmd_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_cmd", 1);


  reset_sub_ = node_handle.subscribe("/franka_control/error_recovery/goal", 1,
    &TuneForcePid::reset_callback, this);
  force_torque_ref_ = node_handle.subscribe("/ft_ref", 1,
    &TuneForcePid::ft_ref_callback, this);

  ori_pid_roll_msg_pub_ = node_handle.advertise<franka_force_control::PIDController>("ori_pid_roll_msg", 1);
  ori_pid_pitch_msg_pub_ = node_handle.advertise<franka_force_control::PIDController>("ori_pid_pitch_msg", 1);
  ori_pid_yaw_msg_pub_ = node_handle.advertise<franka_force_control::PIDController>("ori_pid_yaw_msg", 1);

  const char *wrench_pid_topics[6] = { "/wrench_pid_fx", "/wrench_pid_fy", "/wrench_pid_fz", 
                         "/wrench_pid_tx", "/wrench_pid_ty", "/wrench_pid_tz" }; 
  
  for (int i = 0; i < 6; i++ ){
      wrench_pid_pub_[i] = node_handle.advertise<franka_force_control::PIDController>(wrench_pid_topics[i], 1);
  }

  return true;
}

void TuneForcePid::reset_callback(
  const franka_control::ErrorRecoveryActionGoal&  msg) {
  f_err_int_.setZero();
}

void TuneForcePid::ft_ref_callback(
  const geometry_msgs::WrenchStamped::ConstPtr& msg){

  target_x_ = msg->wrench.force.x;
  target_y_ = msg->wrench.force.y;
  target_z_ = msg->wrench.force.z;

  target_tx_ = msg->wrench.torque.x;
  target_ty_ = msg->wrench.torque.y;
  target_tz_ = msg->wrench.torque.z;
  
}


void TuneForcePid::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();

  f_err_prev_.setZero();
  f_err_int_.setZero();

  median_size_ = 99;
  filter_x_.init(median_size_);
  filter_y_.init(median_size_);
  filter_z_.init(median_size_);

  for (int i = 0; i < 6; i++) {
    force_filter_[i].init(median_size_);
    force_filtered_[i] = 0.;
  }


  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas_init(force_meas_array.data());
  force_meas_init_ = force_meas_init;


}

void TuneForcePid::update(const ros::Time& /*time*/, const ros::Duration& period) {
  
  franka::RobotState robot_state = state_handle_->getRobotState();

  franka::RobotMode mode_ = robot_state.robot_mode;

  std::array<double, 16> T_base_arr = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 4, 4>> T_base(T_base_arr.data());
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(0) = desired_x_;
  desired_force_torque(1) = desired_y_;
  desired_force_torque(2) = desired_z_;
  desired_force_torque(3) = desired_tx_;
  desired_force_torque(4) = desired_ty_;
  desired_force_torque(5) = desired_tz_;

  tau_ext = tau_measured - gravity - tau_ext_initial_;

  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas_no_filt(force_meas_array.data());

  Eigen::Matrix<double, 6, 1> force_cor;
  force_cor = jacobian * coriolis;

  for (int i = 0; i < 6; i++){
    force_filtered_[i] = force_filter_[i].filter(force_meas_array[i] 
       - force_meas_init_[i]) - force_cor[i];
    force_meas_array[i] = force_filtered_[i];
  }
  
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas(force_meas_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_des(desired_force_torque.data());

  Eigen::Matrix<double, 6, 1> force_pid, force_ref;
  force_ref.setZero();


  if (mode_ == franka::RobotMode::kMove && period.toSec() > 0.) {


    float orientation_meas[3];
    Eigen::Matrix4d T_base_4d;
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 4; j++ ) {
        T_base_4d(i,j) = T_base(i,j);
      }
    }
    getAnglesFromRotationTranslationMatrix(T_base_4d, orientation_meas);

    float dt = period.toSec();

    wrapErrToPi(orientation_meas);

    const int N = 5; // repeat N times
    const int start = -3;
    const int duration = 3;
    const int interval = 5;
    
    sequence seq(start, duration, interval, N);

    force_ref[0] = 0.0;
    force_ref[1] = 0.0;
    force_ref[2] = 0.0;

    force_ref[3] = orientation_pid_roll.compute((float)target_tx_, orientation_meas[0], dt);
    force_ref[4] = orientation_pid_pitch.compute((float)target_ty_, orientation_meas[1], dt);
    force_ref[5] = orientation_pid_yaw.compute((float)target_tz_, orientation_meas[2], dt);

    franka_force_control::PIDController pid_msg;
    orientation_pid_roll.create_msg(pid_msg);
    ori_pid_roll_msg_pub_.publish(pid_msg);

    orientation_pid_pitch.create_msg(pid_msg);
    ori_pid_pitch_msg_pub_.publish(pid_msg);

    orientation_pid_yaw.create_msg(pid_msg);
    ori_pid_yaw_msg_pub_.publish(pid_msg);


    for (int i = 0; i < 6; i++ ){
      force_pid[i] = wrench_pid[i].compute(force_ref[i], force_meas[i], dt);
    }

    for (int i = 0; i < 6; i++ ){
      wrench_pid[i].create_msg(pid_msg);
      wrench_pid_pub_[i].publish(pid_msg);
    }

    filter_new_params();

  }
  else {
    desired_x_ = force_meas[0];
    desired_y_ = force_meas[1];
    desired_z_ = force_meas[2];
    target_x_ = 0.;
    target_y_ = 0.;
    target_z_ = 0.;

    force_des = force_meas;

    f_err_prev_.setZero();
    f_err_int_.setZero();

    force_pid = force_des; 
  }


  Eigen::VectorXd tau_pid(7), tau_cmd_pid(7);

  Eigen::Matrix<double, 6, 7> jacobian_reduced;
  jacobian_reduced.setZero();
  jacobian_reduced.block<3,4>(0,0) = jacobian.block<3,4>(0,0);
  jacobian_reduced.block<3,3>(3,4) = jacobian.block<3,3>(3,4);

  // tau_pid << jacobian_reduced.transpose() * (force_pid);

  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian_reduced, jacobian_pinv);

  tau_pid << jacobian_pinv * (force_pid);

  tau_cmd_pid << saturateTorqueRate(tau_pid, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd_pid(i));
  }

  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;

  debug_publish_tau(tau_ext, tau_d, tau_cmd_pid);

}

void TuneForcePid::debug_publish_tau(Eigen::VectorXd tau_ext, Eigen::VectorXd tau_d, Eigen::VectorXd tau_cmd_pid){

  std_msgs::Float32MultiArray array_ext, array_d, array_cmd;
  for(int i = 0; i < 7; i++) {
    array_ext.data.push_back(tau_ext[i]);
    array_d.data.push_back(tau_d[i]);
    array_cmd.data.push_back(tau_cmd_pid[i]);
  }
  tau_ext_pub.publish(array_ext);
  tau_d_pub.publish(array_d);
  tau_cmd_pub.publish(array_cmd);

}



void TuneForcePid::wrapErrToPi(float* orientation_meas){

  double err = target_tx_ - orientation_meas[0];
  if (err > 3.141592) {
    target_tx_ = target_tx_ - 2 * 3.141592;
  }
  if (err < - 3.141592) {
    target_tx_ = target_tx_ + 2 * 3.141592;      
  }
  
  // err = target_ty_ - orientation_meas[1];
  // if (err > 3.141592) {
  //   target_ty_ = target_ty_ - 2 * 3.141592;
  // }
  // if (err < - 3.141592) {
  //   target_ty_ = target_ty_ + 2 * 3.141592;      
  // }

  // err = target_tz_ - orientation_meas[2];
  // if (err > 3.141592) {
  //   target_tz_ = target_tz_ - 2 * 3.141592;
  // }
  // if (err < - 3.141592) {
  //   target_tz_ = target_tz_ + 2 * 3.141592;      
  // }
}


void TuneForcePid::desiredMassParamCallback(
    franka_force_control::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  
  if (starting_ == true) {

    config.o_p_x = pid_ori_x(0, 0);
    config.o_i_x = pid_ori_x(0, 1);
    config.o_d_x = pid_ori_x(0, 2);
    config.o_p_y = pid_ori_y(0, 0);
    config.o_i_y = pid_ori_y(0, 1);
    config.o_d_y = pid_ori_y(0, 2);
    config.o_p_z = pid_ori_z(0, 0);
    config.o_i_z = pid_ori_z(0, 1);
    config.o_d_z = pid_ori_z(0, 2);

    config.f_p_x = ft_pid_target(0,0);
    config.f_i_x = ft_pid_target(0,1);
    config.f_d_x = ft_pid_target(0,2);
    config.f_p_y = ft_pid_target(1,0);
    config.f_i_y = ft_pid_target(1,1);
    config.f_d_y = ft_pid_target(1,2);
    config.f_p_z = ft_pid_target(2,0);
    config.f_i_z = ft_pid_target(2,1);
    config.f_d_z = ft_pid_target(2,2);

    config.t_p_x = ft_pid_target(3,0);
    config.t_i_x = ft_pid_target(3,1);
    config.t_d_x = ft_pid_target(3,2);
    config.t_p_y = ft_pid_target(4,0);
    config.t_i_y = ft_pid_target(4,1);
    config.t_d_y = ft_pid_target(4,2);
    config.t_p_z = ft_pid_target(5,0);
    config.t_i_z = ft_pid_target(5,1);
    config.t_d_z = ft_pid_target(5,2);

    config.limit = integrator_limit_;

    starting_ = false;
  }
  else {

    pid_ori_x(0, 0) = config.o_p_x;
    pid_ori_x(0, 1) = config.o_i_x;
    pid_ori_x(0, 2) = config.o_d_x;
    pid_ori_y(0, 0) = config.o_p_y;
    pid_ori_y(0, 1) = config.o_i_y;
    pid_ori_y(0, 2) = config.o_d_y;
    pid_ori_z(0, 0) = config.o_p_z;
    pid_ori_z(0, 1) = config.o_i_z;
    pid_ori_z(0, 2) = config.o_d_z;

    ft_pid_target(0,0) = config.f_p_x;
    ft_pid_target(0,1) = config.f_i_x;
    ft_pid_target(0,2) = config.f_d_x;
    ft_pid_target(1,0) = config.f_p_y;
    ft_pid_target(1,1) = config.f_i_y;
    ft_pid_target(1,2) = config.f_d_y;
    ft_pid_target(2,0) = config.f_p_z;
    ft_pid_target(2,1) = config.f_i_z;
    ft_pid_target(2,2) = config.f_d_z;

    ft_pid_target(3,0) = config.t_p_x;
    ft_pid_target(3,1) = config.t_i_x;
    ft_pid_target(3,2) = config.t_d_x;
    ft_pid_target(4,0) = config.t_p_y;
    ft_pid_target(4,1) = config.t_i_y;
    ft_pid_target(4,2) = config.t_d_y;
    ft_pid_target(5,0) = config.t_p_z;
    ft_pid_target(5,1) = config.t_i_z;
    ft_pid_target(5,2) = config.t_d_z;

    integrator_limit_ = config.limit;

    orientation_pid_roll.set_kp(pid_ori_x(0, 0));
    orientation_pid_roll.set_ki(pid_ori_x(0, 1));
    orientation_pid_roll.set_kd(pid_ori_x(0, 2));

    orientation_pid_pitch.set_kp(pid_ori_y(0, 0));
    orientation_pid_pitch.set_ki(pid_ori_y(0, 1));
    orientation_pid_pitch.set_kd(pid_ori_y(0, 2));

    orientation_pid_yaw.set_kp(pid_ori_z(0, 0));
    orientation_pid_yaw.set_ki(pid_ori_z(0, 1));
    orientation_pid_yaw.set_kd(pid_ori_z(0, 2));

    for (int i = 0; i < 6; i++ ){
      wrench_pid[i].set_kp(ft_pid_target(i,0));
      wrench_pid[i].set_ki(ft_pid_target(i,1));
      wrench_pid[i].set_kd(ft_pid_target(i,2));
    }


  }
}

Eigen::Matrix<double, 7, 1> TuneForcePid::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}


void TuneForcePid::getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
 float *angles)
{
  double r11, r21, r31, r32, r33;
  double roll, pitch, yaw;

  r11 = rotationTranslationMatrix(0,0);
  r21 = rotationTranslationMatrix(1,0);
  r31 = rotationTranslationMatrix(2,0);
  r32 = rotationTranslationMatrix(2,1);
  r33 = rotationTranslationMatrix(2,2);

  roll = atan2(r32, r33);
  pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
  yaw = atan2(r21, r11);

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}


void TuneForcePid::filter_new_params(){
    desired_x_ = filter_gain_ * target_x_ + (1 - filter_gain_) * desired_x_;
    desired_y_ = filter_gain_ * target_y_ + (1 - filter_gain_) * desired_y_;
    desired_z_ = filter_gain_ * target_z_ + (1 - filter_gain_) * desired_z_;
    desired_tx_ = filter_gain_ * target_tx_ + (1 - filter_gain_) * desired_tx_;
    desired_ty_ = filter_gain_ * target_ty_ + (1 - filter_gain_) * desired_ty_;
    desired_tz_ = filter_gain_ * target_tz_ + (1 - filter_gain_) * desired_tz_;
    
    if (fabs(desired_x_) < 1e-4) {
      desired_x_ = 0;
    }
    if (fabs(desired_y_) < 1e-4) {
      desired_y_ = 0;
    }
    if (fabs(desired_z_) < 1e-4) {
      desired_z_ = 0;
    }

    ft_pid = filter_gain_ * ft_pid_target + (1 - filter_gain_) * ft_pid;

    // pid_ori_x = filter_gain_ * pid_ori_x_target + (1 - filter_gain_) * pid_ori_x;
    // pid_ori_y = filter_gain_ * pid_ori_y_target + (1 - filter_gain_) * pid_ori_y;
    // pid_ori_z = filter_gain_ * pid_ori_z_target + (1 - filter_gain_) * pid_ori_z;  
}


}  // namespace franka_force_control

PLUGINLIB_EXPORT_CLASS(franka_force_control::TuneForcePid,
                       controller_interface::ControllerBase)
