// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_force_control/force_ctrl_reconstruct.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <franka/errors.h>

#define LO -1.0
#define HI 1.0

namespace franka_force_control {

bool ForceCtrlReconstruct::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceCtrlReconstruct: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceCtrlReconstruct: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceCtrlReconstruct: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceCtrlReconstruct: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceCtrlReconstruct: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceCtrlReconstruct: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceCtrlReconstruct: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceCtrlReconstruct: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceCtrlReconstruct: Exception getting joint handles: " << ex.what());
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


  if (ros::param::has("/imp_d_")) {
    ros::param::get("/imp_d_", imp_d_);
    target_imp_d_ = imp_d_;
  }
  if (ros::param::has("/imp_k_")) {
    ros::param::get("/imp_k_", imp_k_);
    target_imp_k_ = imp_k_;
  }
  if (ros::param::has("/imp_m_")) {
    ros::param::get("/imp_m_", imp_m_);
    target_imp_m_ = imp_m_;
  }
  if (ros::param::has("/imp_f_")) {
    ros::param::get("/imp_f_", imp_f_);
    target_imp_f_ = imp_f_;
  }
  if (ros::param::has("/imp_scale_")) {
    ros::param::get("/imp_scale_", imp_scale_);
    target_imp_scale_ = imp_scale_;
  }

  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_force_control::desired_mass_paramConfig>>(
      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceCtrlReconstruct::desiredMassParamCallback, this, _1, _2));

  tau_ext_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_ext", 1);
  tau_d_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_d", 1);
  tau_cmd_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_cmd", 1);

  force_ext_pub = node_handle.advertise<std_msgs::Float32MultiArray>("force_ext", 1);
  force_des_pub = node_handle.advertise<std_msgs::Float32MultiArray>("force_des", 1);
  force_pid_pub = node_handle.advertise<std_msgs::Float32MultiArray>("force_pid", 1);
  force_nof_pub = node_handle.advertise<std_msgs::Float32MultiArray>("force_nof", 1);
  force_ref_pub = node_handle.advertise<std_msgs::Float32MultiArray>("force_ref", 1);

  force_cor_pub = node_handle.advertise<std_msgs::Float32MultiArray>("force_cor", 1);

  pos_ref_glob_pub = node_handle.advertise<std_msgs::Float32MultiArray>("pos_ref_global", 1);  
  pos_glob_pub = node_handle.advertise<std_msgs::Float32MultiArray>("pos_global", 1);  

  reset_sub_ = node_handle.subscribe("/franka_control/error_recovery/goal", 1,
    &ForceCtrlReconstruct::reset_callback, this);
  force_torque_ref_ = node_handle.subscribe("/ft_ref", 1,
    &ForceCtrlReconstruct::ft_ref_callback, this);
  impedance_pos_ref_ = node_handle.subscribe("/exploration_direction", 1, 
    &ForceCtrlReconstruct::imp_pos_ref_callback, this);

  return true;
}

void ForceCtrlReconstruct::reset_callback(
  const franka_control::ErrorRecoveryActionGoal&  msg) {
  f_err_int_.setZero();
}

void ForceCtrlReconstruct::ft_ref_callback(
  const geometry_msgs::WrenchStamped::ConstPtr& msg){

  target_x_ = msg->wrench.force.x;
  target_y_ = msg->wrench.force.y;
  target_z_ = msg->wrench.force.z;

  target_tx_ = msg->wrench.torque.x;
  target_ty_ = msg->wrench.torque.y;
  target_tz_ = msg->wrench.torque.z;
  
}

void ForceCtrlReconstruct::imp_pos_ref_callback(
  const geometry_msgs::Point::ConstPtr& msg) {

  loc_d_x_ = msg->x;
  loc_d_y_ = msg->y;
  loc_d_z_ = msg->z;


}


void ForceCtrlReconstruct::starting(const ros::Time& /*time*/) {
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

  ft_pid.setZero();
  
  // impedance open loop init
  std::array<double, 16> T_base_arr = robot_state.O_T_EE_d;
  pos_global_prev_(0) = T_base_arr[12];
  pos_global_prev_(1) = T_base_arr[13];
  pos_global_prev_(2) = T_base_arr[14];

  vel_global_prev_(0) = 0.;
  vel_global_prev_(1) = 0.;
  vel_global_prev_(2) = 0.;


  dXglobal_.setZero();
  dXglobal_(0) = pos_global_prev_(0);
  dXglobal_(1) = pos_global_prev_(1);
  dXglobal_(2) = pos_global_prev_(2);
  dXglobal_(3) = 1.;

  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas_init(force_meas_array.data());
  force_meas_init_ = force_meas_init;

  srand (time(NULL));
  wiggle_timer_ = 0;
  wiggle_moments_.setZero();

  posGlEE_prev_.setZero();



}

void ForceCtrlReconstruct::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  franka::RobotMode mode_ = robot_state.robot_mode;

  std::array<double, 16> T_base_arr = robot_state.O_T_EE;

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

  // force_meas_array[5] = tau_ext[6];

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

  Eigen::Matrix<double, 6, 1> force_pid, force_imp;
  force_imp.setZero();

  if (mode_ == franka::RobotMode::kMove && period.toSec() > 0.) {

    force_imp = ForceCtrlReconstruct::impedanceOpenLoop(period, force_meas);

    force_pid = ForceCtrlReconstruct::PID(force_meas, force_des + force_imp, period);
    // force_pid = ForceCtrlReconstruct::PID(force_meas, force_des , period);

    // Update signals changed online through dynamic reconfigure
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

    k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
    k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;

    ft_pid = filter_gain_ * ft_pid_target + (1 - filter_gain_) * ft_pid;

    imp_d_ = filter_gain_ * target_imp_d_ + (1 - filter_gain_) * imp_d_; 
    imp_k_ = filter_gain_ * target_imp_k_ + (1 - filter_gain_) * imp_k_; 
    imp_m_ = filter_gain_ * target_imp_m_ + (1 - filter_gain_) * imp_m_; 
    imp_f_ = filter_gain_ * target_imp_f_ + (1 - filter_gain_) * imp_f_; 

    imp_scale_ = filter_gain_ * target_imp_scale_ + (1 - filter_gain_) * imp_scale_; 

    imp_d_ = target_imp_d_;
    imp_k_ = target_imp_k_;
    imp_m_ = target_imp_m_;
    imp_f_ = target_imp_f_;

    imp_scale_ = target_imp_scale_;


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

  tau_pid << jacobian_reduced.transpose() * (force_pid);
  // tau_pid << jacobian.transpose() * (force_pid);

  tau_cmd_pid << saturateTorqueRate(tau_pid, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd_pid(i));
  }

  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  std_msgs::Float32MultiArray array_ext, array_d, array_cmd;
  for(int i = 0; i < 7; i++) {
    array_ext.data.push_back(tau_ext[i]);
    array_d.data.push_back(tau_d[i]);
    array_cmd.data.push_back(tau_cmd_pid[i]);
  }
  tau_ext_pub.publish(array_ext);
  tau_d_pub.publish(array_d);
  tau_cmd_pub.publish(array_cmd);

  std_msgs::Float32MultiArray array_force, array_force_des, array_force_pid, array_no_filt, array_force_all;
  std_msgs::Float32MultiArray array_coriolis;  

  for(int i = 0; i < 6; i++) {
    array_no_filt.data.push_back(force_imp[i]);
    array_force.data.push_back(force_meas[i]);
    array_force_des.data.push_back(desired_force_torque[i]);
    array_force_pid.data.push_back(force_pid[i]);
    array_force_all.data.push_back(force_pid[i] + force_imp[i]);

    array_coriolis.data.push_back(force_cor[i]);

  }
  force_ext_pub.publish(array_force);
  force_des_pub.publish(array_force_des);
  force_pid_pub.publish(array_force_pid);
  force_nof_pub.publish(array_no_filt);
  force_ref_pub.publish(array_force_all);

  force_cor_pub.publish(array_coriolis);
}

Eigen::Matrix<double, 6, 1> ForceCtrlReconstruct::PID(
  Eigen::Matrix<double, 6, 1> measured, 
  Eigen::Matrix<double, 6, 1> desired,
  const ros::Duration& period){

  // Eigen::Matrix<double, 6, 1> force_err = force_des - force_meas;
  Eigen::Matrix<double, 6, 1> force_err;
  force_err.setZero();
  for(int i = 0; i < 6; i++){
    force_err[i] = desired[i] - measured[i];
  }

  Eigen::Matrix<double, 6, 1> d_force_err = force_err - f_err_prev_;
  f_err_prev_ = force_err;

  f_err_int_ += period.toSec() * force_err;
  f_err_int_ = ForceCtrlReconstruct::antiWindup(f_err_int_);

  Eigen::Matrix<double, 6, 1> u_p, u_i, u_d;
  u_p.setZero();
  u_i.setZero();
  u_d.setZero();

  for (int i = 0; i < 6; i++) {
    u_p[i] = ft_pid(i, 0) * force_err[i];
    u_i[i] = ft_pid(i, 1) * f_err_int_[i];
    u_d[i] = ft_pid(i, 2) * d_force_err[i];
  }
  
  Eigen::Matrix<double, 6, 1> pid_out;
  pid_out = desired + u_p + u_i + u_d;

  return pid_out;
}

Eigen::Matrix<double, 6, 1> ForceCtrlReconstruct::antiWindup(Eigen::Matrix<double, 6, 1> u){
  for(int i = 0; i < 6; i++){

    if (u[i] > integrator_limit_) 
      u[i] = integrator_limit_;
    else if (u[i] < - integrator_limit_)
      u[i] = - integrator_limit_;
  }
  return u;
}


Eigen::Matrix<double, 6, 1> ForceCtrlReconstruct::impedanceOpenLoop(
              const ros::Duration& period, Eigen::Matrix<double, 6, 1> f_ext){


  Eigen::Matrix<double, 6, 1> force_torque;
  Eigen::Matrix<double, 3, 1> force;

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 16> T_base_arr = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 4, 4>> T_base(T_base_arr.data());

  Eigen::Matrix<double, 4, 1> dXlocal, dXglobal;
  dXlocal.setZero();
  dXglobal.setZero();

  dXlocal(0) = loc_d_x_;
  dXlocal(1) = loc_d_y_;
  dXlocal(2) = loc_d_z_;
  dXlocal(3) = 1.0;
  
  Eigen::Matrix<double, 3, 1> posGlobal, velGlobal, accGlobal;
  Eigen::Matrix<double, 4, 1> posGlobal4;
  for(int i = 0; i < 3; i++) {
    posGlobal(i) = T_base(i,3);
    posGlobal4(i) = T_base(i,3);
  }
  posGlobal4(3) = 0.0;

  dXglobal = T_base * dXlocal - posGlobal4;

  Eigen::Matrix<double, 3, 1> velRefLoc, velRefGlob, accRefGlob;
  double dxAmpl{0.};
  for (int i = 0; i < 3; i++){
    dxAmpl += pow(dXlocal(i), 2);
    velRefLoc(i) = dXlocal(i);
  } 
  dxAmpl = sqrt(dxAmpl);
  if (dxAmpl > 0.1) {
    velRefLoc = 0.2 * velRefLoc / dxAmpl;
  }

  Eigen::Matrix<double, 3, 3> R_base;
  R_base = T_base.block<3,3>(0,0);

  velRefGlob = R_base * velRefLoc;
  for (int i = 0; i < 3; i++) {
    // velRefGlob[i] = 0.0;
    accRefGlob[i] = 0.;
  }
  

  velGlobal = (posGlobal - pos_global_prev_) / period.toSec();
  accGlobal = (velGlobal - vel_global_prev_) / period.toSec();


  float velAmpl{0.}, impRefAmpl{0.};
  for (int i = 0; i < 3; i++) {
    velAmpl += pow(velGlobal[i], 2.);
    impRefAmpl += pow(dXlocal[i], 2.);
  }
  velAmpl = sqrt(velAmpl);
  impRefAmpl = sqrt(impRefAmpl);

  Eigen::Matrix<double, 3, 1> momentsLoc, moments;
  moments.setZero();
  if (impRefAmpl > 0.1) {
    momentsLoc = wiggle(velAmpl, loc_d_x_);
    moments = R_base * momentsLoc;
  }

  // force = imp_scale_ * (- imp_f_ * f_ext.block<3,1>(0,0)
  //         + imp_m_ * (accRefGlob - accGlobal)  
  //         + imp_d_ * (velRefGlob - velGlobal) 
  //         + imp_k_ * dXglobal.block(0,0,3,1));

  force = (imp_m_ / imp_scale_ - 1.0) * f_ext.block<3,1>(0,0) 
              // - imp_m_ * accRefGlob
              - (imp_m_ / imp_scale_) * (imp_d_ * (velGlobal - velRefGlob)
                - imp_k_ * dXglobal.block(0,0,3,1));

  force_torque.setZero();

  for(int i = 0; i < 3; i++) {
    force_torque[i] = force[i];
    // force_torque[i+3] = moments[i];
  }

  pos_global_prev_ = posGlobal;
  vel_global_prev_ = velGlobal;

  std_msgs::Float32MultiArray array_dxglob, array_posglob;
  for(int i = 0; i < 3; i++) {
    array_dxglob.data.push_back(dXglobal[i]);
    array_dxglob.data.push_back(velGlobal[i]);
    array_posglob.data.push_back(posGlobal[i]);
  }
  array_dxglob.data.push_back(velAmpl);
  pos_ref_glob_pub.publish(array_dxglob);
  pos_glob_pub.publish(array_posglob);

  return force_torque;

}


Eigen::Matrix<double, 3, 1> ForceCtrlReconstruct::wiggle(float velocity_amplitude, float loc_d_x_) {
  
  if (wiggle_timer_ >= 200) {
    Eigen::Matrix<double, 3, 1> rand_vec;
    float rand_vec_ampl = 0.;
    for(int i = 0; i < 3; i++){
      rand_vec[i] = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
      // rand_vec_ampl += pow(rand_vec[i], 2.);
    }


    float prob = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX));

    float predznak = -1.;
    
    if (loc_d_x_ < 0.) {
      predznak *= -1.;
    }
    if (prob > 0.4) {
      predznak *= -1.;
    }


    rand_vec[0] = 0.3 * rand_vec[0];
    rand_vec[1] = predznak * fabs(rand_vec[1]);
    rand_vec[2] = 0.3 * rand_vec[2];
    

    for(int i = 0; i < 3; i++){
      rand_vec_ampl += pow(rand_vec[i], 2.);
    }
    rand_vec_ampl = sqrt(rand_vec_ampl);
    rand_vec /= rand_vec_ampl;

    // std::cout << rand_vec.transpose() << std::endl;

    // // rand_vec *= 0.1;
    // // rand_vec -= wiggle_moments_;
    // rand_vec_ampl = 0.;
    // for(int i = 0; i < 3; i++){
    //   rand_vec[i] = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    //   rand_vec_ampl += pow(rand_vec[i], 2.);
    // }
    // rand_vec_ampl = sqrt(rand_vec_ampl);
    // rand_vec /= rand_vec_ampl;

    // std::cout << rand_vec.transpose() << std::endl;  

    float wiggle_amplitude = 0.;
    float scale = 1.5;


    wiggle_amplitude = (1. - velocity_amplitude / 0.2);
    if (wiggle_amplitude < 0.) {
      wiggle_amplitude = 0.;
    }
    wiggle_amplitude = scale * pow(wiggle_amplitude, 2);

    wiggle_moments_ = rand_vec * wiggle_amplitude;
    wiggle_timer_ = 0;
  

    std_msgs::Float32MultiArray array_dxglob;
    array_dxglob.data.push_back(velocity_amplitude);
    array_dxglob.data.push_back(velocity_amplitude-vel_ampl_prev_);
    // pos_ref_glob_pub.publish(array_dxglob);
    vel_ampl_prev_ = velocity_amplitude;

  }

  wiggle_timer_++;
  return wiggle_moments_;

}


void ForceCtrlReconstruct::desiredMassParamCallback(
    franka_force_control::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  
  if (starting_ == true) {
    config.k_p = target_k_p_;
    config.k_i = target_k_i_;


    config.imp_d = imp_d_;
    config.imp_k = imp_k_;
    config.imp_m = imp_m_;
    config.imp_f = imp_f_;
    config.scale = imp_scale_;

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

    target_imp_f_ = config.imp_f;
    target_imp_m_ = config.imp_m;
    target_imp_d_ = config.imp_d;
    target_imp_k_ = config.imp_k;
    target_imp_scale_ = config.scale;

    target_k_p_ = config.k_p;
    target_k_i_ = config.k_i;

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
  }
}

Eigen::Matrix<double, 7, 1> ForceCtrlReconstruct::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_force_control

PLUGINLIB_EXPORT_CLASS(franka_force_control::ForceCtrlReconstruct,
                       controller_interface::ControllerBase)
