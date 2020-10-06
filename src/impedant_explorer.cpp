// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_force_control/impedant_explorer.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <franka/errors.h>

#include "pseudo_inversion.h"
// #include "pulse.h"

#define LO -1.0
#define HI 1.0
#define N_ORDER 3
#define MIN_DIST_ 0.1
#define MAX_HIST_LEN_ 15
#define MIN_DIST_THRES_ 0.01

namespace franka_force_control {

bool ImpedantExplorer::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ImpedantExplorer: Assuming SOFT gripper!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ImpedantExplorer: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ImpedantExplorer: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ImpedantExplorer: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ImpedantExplorer: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ImpedantExplorer: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ImpedantExplorer: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ImpedantExplorer: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ImpedantExplorer: Exception getting joint handles: " << ex.what());
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

  for (int i = 0; i < 6; i++ ){
    wrench_pid[i].set_kp(ft_pid_target(i,0));
    wrench_pid[i].set_ki(ft_pid_target(i,1));
    wrench_pid[i].set_kd(ft_pid_target(i,2));
    wrench_pid[i].set_lim_high(integrator_limit_);
    wrench_pid[i].set_lim_low(-integrator_limit_);
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
  if (ros::param::has("/imp_scale_m_")) {
    ros::param::get("/imp_scale_m_", imp_scale_m_);
    target_imp_scale_m_ = imp_scale_m_;
  }
  if (ros::param::has("/imp_scale_m_i_")) {
    ros::param::get("/imp_scale_m_i_", imp_scale_m_i_);
    target_imp_scale_m_i_ = imp_scale_m_i_;
  }

  tau_ext_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_ext", 1);
  tau_d_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_d", 1);
  tau_cmd_pub = node_handle.advertise<std_msgs::Float32MultiArray>("tau_cmd", 1);

  force_g_pub = node_handle.advertise<geometry_msgs::WrenchStamped>("/franka_state_controller/F_g", 1);
  force_c_pub = node_handle.advertise<geometry_msgs::WrenchStamped>("/franka_state_controller/F_c", 1);

  reset_sub_ = node_handle.subscribe("/franka_control/error_recovery/goal", 1,
    &ImpedantExplorer::reset_callback, this);
  force_torque_ref_ = node_handle.subscribe("/ft_ref", 1,
    &ImpedantExplorer::ft_ref_callback, this);

  impedance_pos_ref_ = node_handle.subscribe("/exploration_direction", 1, 
    &ImpedantExplorer::imp_pos_ref_callback, this);

  const char *wrench_pid_topics[6] = { "/wrench_pid_fx", "/wrench_pid_fy", "/wrench_pid_fz", 
                         "/wrench_pid_tx", "/wrench_pid_ty", "/wrench_pid_tz" }; 
  
  for (int i = 0; i < 6; i++ ){
      wrench_pid_pub_[i] = node_handle.advertise<franka_force_control::PIDController>(wrench_pid_topics[i], 1);
  }



  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_force_control::desired_mass_paramConfig>>(
      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ImpedantExplorer::desiredMassParamCallback, this, _1, _2));


  marker_pos_ = node_handle.advertise<visualization_msgs::Marker>("/franka_pos", 1);
  marker_pip_ = node_handle.advertise<visualization_msgs::Marker>("/pipe_ori", 1);


  return true;
}

void ImpedantExplorer::reset_callback(
  const franka_msgs::ErrorRecoveryActionGoal&  msg) {
  f_err_int_.setZero(); 
  polyfit_history = 0;
  flag_reset_ = 0;
  moments_integrate_.setZero();
}

void ImpedantExplorer::ft_ref_callback(
  const geometry_msgs::WrenchStamped::ConstPtr& msg){

  target_x_ = msg->wrench.force.x;
  target_y_ = msg->wrench.force.y;
  target_z_ = msg->wrench.force.z;

  target_tx_ = msg->wrench.torque.x;
  target_ty_ = msg->wrench.torque.y;
  target_tz_ = msg->wrench.torque.z;
  
}

void ImpedantExplorer::imp_pos_ref_callback(
  const geometry_msgs::Point::ConstPtr& msg) {

  loc_d_x_ = msg->x;
  loc_d_y_ = msg->y;
  loc_d_z_ = msg->z;


}

void ImpedantExplorer::starting(const ros::Time& /*time*/) {


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


  // impedance open loop init
  std::array<double, 16> T_base_arr = robot_state.O_T_EE_d;
  pos_global_prev_ << T_base_arr[12], T_base_arr[13], T_base_arr[14];
  vel_global_prev_ << 0., 0., 0.;
  dXglobal_ << pos_global_prev_(0), pos_global_prev_(1), pos_global_prev_(2), 1.;

  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas_init(force_meas_array.data());
  force_meas_init_ = force_meas_init;

  srand (time(NULL));
  wiggle_timer_ = 0;
  wiggle_moments_.setZero();

  posGlEE_prev_.setZero();

  history_pos_.setZero(3,MAX_HIST_LEN_);

  history_pos_.block(0,MAX_HIST_LEN_-1,3,1) = pos_global_prev_;

  incline_.setZero(3,1);
  target_incline_.setZero(3,1);

  cummulative_dist_.setZero(MAX_HIST_LEN_);
  distances_.setZero(MAX_HIST_LEN_);

  moments_integrate_.setZero(3);  


}

void ImpedantExplorer::update(const ros::Time& /*time*/, const ros::Duration& period) {

  franka::RobotState robot_state = state_handle_->getRobotState();

  franka::RobotMode mode_ = robot_state.robot_mode;

  std::array<double, 16> T_base_arr = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 4, 4>> T_base(T_base_arr.data());
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas_no_filt(force_meas_array.data());

  for (int i = 0; i < 6; i++){
    force_filtered_[i] = force_filter_[i].filter(force_meas_array[i] 
       - force_meas_init_[i]);// - force_cor[i];
    force_meas_array[i] = force_filtered_[i];
  }
  
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_meas(force_meas_array.data());

  Eigen::VectorXd desired_force_torque(6);
  desired_force_torque << desired_x_, desired_y_, desired_z_, desired_tx_, desired_ty_, desired_tz_;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_des(desired_force_torque.data());

  Eigen::Matrix<double, 6, 1> force_pid, force_imp, force_ref;
  force_ref.setZero();
  force_imp.setZero();
  // force_des.setZero();
  force_pid.setZero();

  Eigen::VectorXd tau_cmd(7);
  
  Eigen::VectorXd position(3), incline(3);
  Eigen::VectorXd pos_loc(4), pos_glob(4);
  pos_loc << 0, -0.03, 0.03, 1;
  pos_glob = T_base * pos_loc;
  position = pos_glob.block(0,0,3,1);


  // Eigen::Vector3d rot_vel(0.,0.,0.);

  // Eigen::Vector3d globxcur, globycur, globzcur;
  // Eigen::Vector3d globxref, globyref, globzref;
  // Eigen::Matrix3d r_cur, r_ref, r_transform;
  // //curr 
  // locy << 0, -1, 0;
  // Eigen::Matrix3d R_base;
  // R_base = T_base.block<3,3>(0,0);

  // globycur = R_base * locy;
  // locx << 1, 0, 0;
  // globxcur = R_base * locx;

  // globzcur = globxcur.cross(globycur);

  // globyref = incline_;
  // globyref.normalize();

  // double ref_frame_score[4] = {-1., -1., -1., -1.};

  // globxref = globyref.cross(globycur);
  // globzref = globxref.cross(globyref);
  // globxref.normalize();
  // globzref.normalize();
  // ref_frame_score[0] = (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

  // globxref = globycur.cross(globyref);
  // globzref = globxref.cross(globyref);
  // globxref.normalize();
  // globzref.normalize();
  // ref_frame_score[1] = (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

  // globzref = globyref.cross(globycur);
  // globxref = globyref.cross(globzref);
  // globxref.normalize();
  // globzref.normalize();
  // ref_frame_score[2] =  (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

  // globzref = globycur.cross(globyref);
  // globxref = globyref.cross(globzref);
  // globxref.normalize();
  // globzref.normalize();
  // ref_frame_score[3] =  (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

  // const int N = sizeof(ref_frame_score) / sizeof(double);
  // int best = std::distance(ref_frame_score, std::max_element(ref_frame_score, ref_frame_score + N));

  // switch (best) {
  //   case 0:
  //     globxref = globyref.cross(globycur);
  //     globzref = globxref.cross(globyref);
  //     break;
  //   case 1:
  //     globxref = globycur.cross(globyref);
  //     globzref = globxref.cross(globyref);
  //     break;
  //   case 2:
  //     globzref = globyref.cross(globycur);
  //     globxref = globyref.cross(globzref);
  //     break;
  //   case 3:
  //     globzref = globycur.cross(globyref);
  //     globxref = globyref.cross(globzref);
  //     break;
  // }
  // globxref.normalize();
  // globzref.normalize();

  // r_cur << globxcur, globycur, globzcur;
  // r_cur.transpose();

  // r_ref << globxref, globyref, globzref;
  // r_ref.transpose();

  // r_transform = r_ref * r_cur.inverse();

  // rot_vel << -r_transform(1,2), r_transform(0,2), -r_transform(0,1);



    // ---------projekcije-------------
    // Eigen::Matrix<float, 3, 1> ref3d, curr3d;
    // ref3d << 1.0, 0.0, 0.0;
    // curr3d << 0.0, 1.0, 0.0;
    // Eigen::Vector3d rot_vel(0.,0.,0.);
    
    // Eigen::Matrix<float, 2, 1> axis;

    // // xy 
    // axis << 1.0, 0.0;
    // Eigen::Matrix<float, 2, 1> ref, curr;
    // ref << ref3d[0], ref3d[1];
    // curr << curr3d[0], curr3d[1];
    // if (ref.norm())
    //   ref.normalize();
    // if (curr.norm())
    //   curr.normalize();
    // if (ref.norm() && curr.norm())
    //   rot_vel[2] = std::acos(ref.dot(axis)) - std::acos(curr.dot(axis)) ;

    // // xz 
    // ref << ref3d[0], r;ef3d[2];
    // curr << curr3d[0], curr3d[2];
    // if (ref.norm())
    //   ref.normalize();
    // if (curr.norm())
    //   curr.normalize();
    // if (ref.norm() && curr.norm())
    //   rot_vel[1] = std::acos(ref.dot(axis)) - std::acos(curr.dot(axis)) ;

    // // yz 
    // ref << ref3d[1], ref3d[2];
    // curr << curr3d[1], curr3d[2];
    // if (ref.norm())
    //   ref.normalize();
    // if (curr.norm())
    //   curr.normalize();
    // if (ref.norm() && curr.norm())
    //   rot_vel[0] = std::acos(ref.dot(axis)) - std::acos(curr.dot(axis)) ;

    // std::cout << ref3d.transpose() << std::endl << std::endl;
    // std::cout << curr3d.transpose() << std::endl << std::endl;
    // std::cout << rot_vel.transpose() << std::endl << std::endl << std::endl;
    // std::cout << "==========" << std::endl << std::endl;
    



  if (mode_ == franka::RobotMode::kMove && period.toSec() > 0.) {


    if (pipe_dir_freq_ == 0) {
      pipe_dir_freq_ = 20;
      incline = ImpedantExplorer::directionPrediction(position);
      target_incline_ = incline;
    }
    else {
      pipe_dir_freq_ -= 1;
    }
    incline_ = 0.9 * incline_ + 0.1 * target_incline_;


    force_imp = ImpedantExplorer::impedanceOpenLoop(period, force_meas, incline_);


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

    force_ref = force_des + force_imp;

    for (int i = 0; i < 6; i++ ){

      force_pid[i] = wrench_pid[i].compute(force_ref[i], force_meas[i], dt);
      // force_pid[i] = wrench_pid[i].compute(force_des[i], force_meas[i], dt);
    }

    franka_force_control::PIDController pid_msg;
    for (int i = 0; i < 6; i++ ){
      wrench_pid[i].create_msg(pid_msg);
      wrench_pid_pub_[i].publish(pid_msg);
    }

    rviz_markers_plot(position);

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
  // pseudoInverse(jacobian_reduced, jacobian_pinv);

  // pseudoInverse(jacobian, jacobian_pinv);
  // std::cout << jacobian_pinv << std::endl;

  jacobian_pinv = jacobian.transpose();

  tau_pid << jacobian_pinv * (force_pid);

  // tau_pid[6] = ref;
  // ramp_ref();
  // tau_pid[joint_torque_ctrl_id] = joint_torque_ref_;

  tau_cmd_pid << saturateTorqueRate(tau_pid, tau_J_d);


  // std::cout << "AAAAA" << std::endl << std::endl << std::endl << std::endl;
  // std::cout << force_des.transpose() << std::endl << std::endl << std::endl << std::endl;
  // std::cout << force_meas.transpose() << std::endl << std::endl << std::endl << std::endl;
  // std::cout << force_pid.transpose() << std::endl << std::endl << std::endl << std::endl;
  // std::cout << tau_cmd_pid.transpose() << std::endl << std::endl << std::endl << std::endl;


  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd_pid(i));
  }

  Eigen::VectorXd tau_d(7), tau_ext(7);
  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;

  // debug_publish_tau(tau_measured, tau_pid, tau_cmd_pid);
  debug_publish_tau(tau_ext, tau_pid, tau_cmd_pid);

  // std::cout << "12" << std::endl << std::endl << std::endl << std::endl;

  // Eigen::Matrix<double, 6, 1> force_cor, force_g;
  // force_cor = jacobian * coriolis;
  // force_g = jacobian * gravity_arrayy;

  geometry_msgs::WrenchStamped f_msg;
  f_msg.header.stamp = ros::Time::now();
  f_msg.wrench.force.x = force_imp[0];
  f_msg.wrench.force.y = force_imp[1];
  f_msg.wrench.force.z = force_imp[2];
  f_msg.wrench.torque.x = force_imp[3];
  f_msg.wrench.torque.y = force_imp[4];
  f_msg.wrench.torque.z = force_imp[5];
  force_g_pub.publish(f_msg);
  // f_msg.header.stamp = ros::Time::now();
  // f_msg.wrench.force.x = force_cor[0];
  // f_msg.wrench.force.y = force_cor[1];
  // f_msg.wrench.force.z = force_cor[2];
  // f_msg.wrench.torque.x = force_cor[3];
  // f_msg.wrench.torque.y = force_cor[4];
  // f_msg.wrench.torque.z = force_cor[5];
  // force_c_pub.publish(f_msg);


  // std::cout << "13" << std::endl << std::endl << std::endl << std::endl;

}


void ImpedantExplorer::debug_publish_tau(Eigen::VectorXd tau_ext, Eigen::VectorXd tau_d, Eigen::VectorXd tau_cmd_pid){

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



void ImpedantExplorer::wrapErrToPi(float* orientation_meas){

  // double err = target_tx_ - orientation_meas[0];
  // if (err > 3.141592) {
  //   target_tx_ = target_tx_ - 2 * 3.141592;
  // }
  // if (err < - 3.141592) {
  //   target_tx_ = target_tx_ + 2 * 3.141592;      
  // }
  
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


void ImpedantExplorer::desiredMassParamCallback(
    franka_force_control::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  
  if (starting_ == true) {

    config.imp_d = imp_d_;
    config.imp_k = imp_k_;
    config.imp_m = imp_m_;
    config.imp_f = imp_f_;
    config.scale = imp_scale_;
    config.scale_m = imp_scale_m_;
    config.scale_m_i = imp_scale_m_i_;


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
    target_imp_scale_m_ = config.scale_m;
    target_imp_scale_m_i_ = config.scale_m_i;

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

    for (int i = 0; i < 6; i++ ){
      wrench_pid[i].set_kp(ft_pid_target(i,0));
      wrench_pid[i].set_ki(ft_pid_target(i,1));
      wrench_pid[i].set_kd(ft_pid_target(i,2));
    }


  }
}

Eigen::Matrix<double, 7, 1> ImpedantExplorer::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    // if ((difference > kDeltaTauMax) || (difference < -kDeltaTauMax)) {
    //   std::cout << "ZASICENJE. ref: " << ref << std::endl;
    // }
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void ImpedantExplorer::getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
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


void ImpedantExplorer::filter_new_params(){
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


    // imp_d_ = filter_gain_ * target_imp_d_ + (1 - filter_gain_) * imp_d_; 
    // imp_k_ = filter_gain_ * target_imp_k_ + (1 - filter_gain_) * imp_k_; 
    // imp_m_ = filter_gain_ * target_imp_m_ + (1 - filter_gain_) * imp_m_; 
    // imp_f_ = filter_gain_ * target_imp_f_ + (1 - filter_gain_) * imp_f_; 

    // imp_scale_ = filter_gain_ * target_imp_scale_ + (1 - filter_gain_) * imp_scale_; 

    imp_d_ = target_imp_d_;
    imp_k_ = target_imp_k_;
    imp_m_ = target_imp_m_;
    imp_f_ = target_imp_f_;

    imp_scale_ = target_imp_scale_;

    imp_scale_m_ = target_imp_scale_m_; 
    imp_scale_m_i_ = target_imp_scale_m_i_;  

}




Eigen::Matrix<double, 6, 1> ImpedantExplorer::impedanceOpenLoop(
              const ros::Duration& period, Eigen::Matrix<double, 6, 1> f_ext, Eigen::VectorXd incline){


  Eigen::Matrix<double, 6, 1> force_torque;
  Eigen::Matrix<double, 3, 1> force;

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 16> T_base_arr = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 4, 4>> T_base(T_base_arr.data());

  Eigen::Matrix<double, 4, 1> dXlocal, dXglobal;
  dXlocal.setZero();
  dXglobal.setZero();

  dXlocal << loc_d_x_, loc_d_y_, loc_d_z_, 1.0;

  Eigen::Matrix<double, 3, 1> posGlobal, velGlobal, accGlobal;
  Eigen::Matrix<double, 4, 1> posGlobal4;

  posGlobal << T_base(0,3), T_base(1,3), T_base(2,3);
  posGlobal4 << T_base(0,3), T_base(1,3), T_base(2,3), 0.0;

  dXglobal = T_base * dXlocal - posGlobal4;

  Eigen::Matrix<double, 3, 1> velRefLoc, velRefGlob, accRefGlob;

  velRefLoc = dXlocal.block<3,1>(0,0);
  if (velRefLoc.norm() > 0.0) {
    velRefLoc.normalize();
    velRefLoc = 0.3 * velRefLoc;
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


  // force = imp_scale_ * (- imp_f_ * f_ext.block<3,1>(0,0)
  //         + imp_m_ * (accRefGlob - accGlobal)  
  //         + imp_d_ * (velRefGlob - velGlobal) 
  //         + imp_k_ * dXglobal.block(0,0,3,1));

  force = (imp_m_ / imp_scale_ - 1.0) * f_ext.block<3,1>(0,0) 
          - (imp_m_ / imp_scale_) * (imp_d_ * (velGlobal - velRefGlob)
          - imp_k_ * dXglobal.block(0,0,3,1));


  // std::cout << "stuff" << std::endl;
  // std::cout << f_ext.block<3,1>(0,0) << std::endl;
  // std::cout << velGlobal - velRefGlob  << std::endl << std::endl;
  // std::cout << velRefGlob << std::endl << std::endl << std::endl;
  // std::cout << (imp_m_ / imp_scale_) * imp_k_ * dXglobal[1] << std::endl;


  float velAmpl{0.}, impRefAmpl{0.};
  velAmpl = velGlobal.norm();
  impRefAmpl = dXlocal.block<3,1>(0,0).norm();


  Eigen::Matrix<double, 3, 1> momentsLoc, moments, moments_wiggle;
  moments.setZero();
  moments_wiggle.setZero();
  if (impRefAmpl >= 0.1) {
    momentsLoc = wiggle(velAmpl);
    moments_wiggle = R_base * momentsLoc;
  }

  Eigen::VectorXd refDirLoc(3), refDirGlob(3);


  if (velRefGlob.norm() > 0.) {

    std::cout << "AAA" << std::endl;

    refDirLoc << loc_d_x_, loc_d_y_, loc_d_z_;
    refDirLoc.normalize();
    refDirGlob = R_base * refDirLoc;


    Eigen::Vector3d rot_vel(0.,0.,0.);
    Eigen::Vector3d globxcur, globycur, globzcur;
    Eigen::Vector3d globxref, globyref, globzref;
    Eigen::Matrix3d r_cur, r_ref, r_transform;
    //curr 

    globycur << refDirGlob[0], refDirGlob[1], refDirGlob[2];

    Eigen::Vector3d locx;
    locx << 1, 0, 0;
    globxcur = R_base * locx;

    globzcur = globxcur.cross(globycur);

    globxcur.normalize();
    globycur.normalize();
    globzcur.normalize();

    // std::cout << incline << std::endl << std::endl;
    globyref = incline;

    double ref_frame_score[4] = {-1., -1., -1., -1.};

    globxref = globyref.cross(globycur);
    globzref = globxref.cross(globyref);
    globxref.normalize();
    globzref.normalize();
    ref_frame_score[0] = (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

    globxref = globycur.cross(globyref);
    globzref = globxref.cross(globyref);
    globxref.normalize();
    globzref.normalize();
    ref_frame_score[1] = (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

    globzref = globyref.cross(globycur);
    globxref = globyref.cross(globzref);
    globxref.normalize();
    globzref.normalize();
    ref_frame_score[2] =  (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

    globzref = globycur.cross(globyref);
    globxref = globyref.cross(globzref);
    globxref.normalize();
    globzref.normalize();
    ref_frame_score[3] =  (double) (globxref.dot(globxcur) + globzref.dot(globzcur));

    const int N = sizeof(ref_frame_score) / sizeof(double);
    int best = std::distance(ref_frame_score, std::max_element(ref_frame_score, ref_frame_score + N));

    switch (best) {
      case 0:
        globxref = globyref.cross(globycur);
        globzref = globxref.cross(globyref);
        break;
      case 1:
        globxref = globycur.cross(globyref);
        globzref = globxref.cross(globyref);
        break;
      case 2:
        globzref = globyref.cross(globycur);
        globxref = globyref.cross(globzref);
        break;
      case 3:
        globzref = globycur.cross(globyref);
        globxref = globyref.cross(globzref);
        break;
    }

    if (globyref.norm()) {
      globyref.normalize();
    }
    if (globxref.norm()) {
      globxref.normalize();
    }
    if (globzref.norm()) {
      globzref.normalize();
    }

    r_cur << globxcur, globycur, globzcur;
    r_cur.transpose();

    r_ref << globxref, globyref, globzref;
    r_ref.transpose();

    r_transform = r_ref * r_cur.inverse();

    std::cout << r_ref << std::endl << std::endl << std::endl ;
    std::cout << r_cur << std::endl << std::endl << std::endl ;
    std::cout << r_transform << std::endl << std::endl << std::endl ;


    rot_vel << -r_transform(1,2), r_transform(0,2), -r_transform(0,1);

    // rot_vel = rot_vel + f_ext.block<3,1>(3,0);
    rot_vel[0] += f_ext[3];
    rot_vel[1] += f_ext[4];
    // rot_vel[2] -= f_ext[5];

    std::cout << rot_vel.transpose() << std::endl << std::endl << std::endl;

    moments_integrate_ = moments_integrate_ + rot_vel * period.toSec();
    // moments_integrate_ = moments_integrate_ + (incline - refDirGlob) * period.toSec();

    double integrator_limit_moments = integrator_limit_/2.0;
    for(int i = 0; i < 3; i++) {
      if (moments_integrate_[i] > integrator_limit_moments)
        moments_integrate_[i] = integrator_limit_moments;
      if (moments_integrate_[i] < - integrator_limit_moments)
        moments_integrate_[i] = -integrator_limit_moments;
    }

    // moments = imp_scale_m_ * (incline - refDirGlob) + imp_scale_m_i_ * moments_integrate_;     
    moments = imp_scale_m_ * rot_vel + imp_scale_m_i_ * moments_integrate_;     
  }
  else {
    refDirLoc.setZero();
    refDirGlob.setZero();
    moments.setZero();
  }

  force_torque.setZero();

  for(int i = 0; i < 3; i++) {
    force_torque[i] = force[i];
    // do not update moment references, only force

    // if (gripper_rigid_ == false) {
    //   force_torque[i+3] = moments[i] + 0.5 * moments_wiggle[i];
    // }
    // else
      force_torque[i+3] = moments[i];
      // force_torque[i+3] = moments[i] + moments_wiggle[i];
  }

  pos_global_prev_ = posGlobal;
  vel_global_prev_ = velGlobal;


  return force_torque;

}


Eigen::Matrix<double, 3, 1> ImpedantExplorer::wiggle(float velocity_amplitude) {
  
  if (wiggle_timer_ >= 200) {
    Eigen::Matrix<double, 3, 1> rand_vec;
    float rand_vec_ampl = 0.;
    for(int i = 0; i < 3; i++){
      rand_vec[i] = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
      // rand_vec_ampl += pow(rand_vec[i], 2.);
    }

    float prob = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX));

    float predznak = -1.;
    
    if (loc_d_x_ != 0.) {
      if (loc_d_x_ < 0.) {
        predznak *= -1.;
      }
      if (prob > 0.4) {
        predznak *= -1.;
      }
      rand_vec[0] = 0.3 * rand_vec[0];
      rand_vec[1] = predznak * fabs(rand_vec[1]);
      rand_vec[2] = 0.3 * rand_vec[2];
    }
    if (loc_d_y_ != 0.) {
      if (loc_d_y_ > 0.) {
        predznak *= -1.;
      }
      if (prob > 0.4) {
        predznak *= -1.;
      }
      rand_vec[0] = predznak * fabs(rand_vec[0]);
      rand_vec[1] = 0.3 * (rand_vec[1]);
      rand_vec[2] = 0.3 * rand_vec[2];
    }

    for(int i = 0; i < 3; i++){
      rand_vec_ampl += pow(rand_vec[i], 2.);
    }
    rand_vec_ampl = sqrt(rand_vec_ampl);
    rand_vec /= rand_vec_ampl;

    float wiggle_amplitude = 0.;
    float scale = 1.5;

    wiggle_amplitude = (1. - velocity_amplitude / 0.2);
    if (wiggle_amplitude < 0.) {
      wiggle_amplitude = 0.;
    }
    wiggle_amplitude = scale * pow(wiggle_amplitude, 2);

    wiggle_moments_ = rand_vec * wiggle_amplitude;
    wiggle_timer_ = 0;
  
    // std_msgs::Float32MultiArray array_dxglob;
    // array_dxglob.data.push_back(velocity_amplitude);
    // array_dxglob.data.push_back(velocity_amplitude-vel_ampl_prev_);
    // pos_ref_glob_pub.publish(array_dxglob);
    vel_ampl_prev_ = velocity_amplitude;
  }

  wiggle_timer_++;
  return wiggle_moments_;
}


Eigen::VectorXd ImpedantExplorer::directionPrediction(Eigen::VectorXd position) {

  Eigen::MatrixXd new_dist_vec;

  new_dist_vec = history_pos_.block(0,MAX_HIST_LEN_-1,3,1) - position;
  double new_dist = new_dist_vec.norm();

  int n = polyfit_history; // number of points
  int len = n - 1; // largest index

  if (new_dist > MIN_DIST_THRES_) {

    history_pos_.block(0,0,3,MAX_HIST_LEN_-1) = history_pos_.block(0,1,3,MAX_HIST_LEN_-1);
    history_pos_.block(0,MAX_HIST_LEN_-1,3,1) = position;    
    if (polyfit_history < MAX_HIST_LEN_) {
      polyfit_history += 1;
    }
    distances_.block(0, 0, MAX_HIST_LEN_-1, 1) = cummulative_dist_.block(1, 0, MAX_HIST_LEN_-1, 1);
    distances_(MAX_HIST_LEN_-1) = new_dist;


    n = polyfit_history; // number of points
    len = n - 1; // largest index

    cummulative_dist_[len] = 0.;
    // po svim tockama osim zadnje - najnovije
    for (int i = 1; i < n; i++) {
      //popunjavanje od najnovije otkraja prema 0 (zapravo 1)
      // cummulative distance -  oldest 0 --> newest polyfit_history 
      cummulative_dist_[len - i] = cummulative_dist_[len - (i - 1)] + distances_[len - (i - 1)];
    }

  }

  bool standing_still = false;

  Eigen::VectorXd incline;
  incline.setZero(3,1);

  if (not standing_still) {

    Eigen::VectorXd dimension(polyfit_history);
    Eigen::VectorXd timeparam(polyfit_history);

    for (int i = 0; i < n; i++) {
      timeparam(i) = cummulative_dist_(i) / cummulative_dist_(0);
    }

    Eigen::MatrixXd coeffs(N_ORDER+1, 3);
    coeffs.setZero(N_ORDER+1, 3);
    
    for(int i = 0; i < 3; i++){
      dimension = history_pos_.block(i, MAX_HIST_LEN_ - n, 1, n).transpose();
      coeffs.col(i) = ImpedantExplorer::polyfit(timeparam, dimension, N_ORDER);
      // za svaku prostornu dimenziju smjer je derivacija u t=0 
      // jer je vrijeme okrenuto tako da je t=0 zadnji trenutak
      // tako da je smjer tj derivacija koef uz x^1 u polinomu 
      // tj a1 u zapisu a0 x^0 + a1 x^1 + a2 x^2 + ...
      incline(i) = - coeffs(2,i);
    }


    if (incline.norm() != 0.)
      incline = incline / incline.norm();

    if (std::isnan(incline.norm())) {
      incline.setZero(3,1);
      // std::cout << "NAN in inclination calculation" << std::endl;
    }

    if (flag_reset_ < 10){
      flag_reset_ += 1;

    }
  }

  return incline;

}

Eigen::VectorXd ImpedantExplorer::polyfit(Eigen::VectorXd t, Eigen::VectorXd y,int order) 
{

  // Initialize the coefficient matrix of \eqref{eq:polyfitlse}
  Eigen::MatrixXd A = Eigen::MatrixXd::Ones(t.size(),order + 1);
  for (unsigned j = 1; j < order + 1; ++j) 
    A.col(j) = A.col(j - 1).cwiseProduct(t);
  // Use \eigen's built-in least squares solver, see \cref{cpp:lsqsolveeigen}
  Eigen::VectorXd coeffs = A.householderQr().solve(y);
  // leading coefficients have low indices.
  return coeffs.reverse();
}



void ImpedantExplorer::rviz_markers_plot(Eigen::VectorXd position){
  if (count_markers_ == 100) {
    marker_id_ += 1;
    visualization_msgs::Marker marker_pos, marker_pip;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker_pos.header.frame_id = "/panda_link0";
    marker_pos.header.stamp = ros::Time::now();
    marker_pos.ns = "position";
    marker_pos.id = marker_id_;
    marker_pos.type = shape;
    marker_pos.action = visualization_msgs::Marker::ADD;
    marker_pos.pose.position.x = position[0];
    marker_pos.pose.position.y = position[1];
    marker_pos.pose.position.z = position[2];
    marker_pos.pose.orientation.x = 0.0;
    marker_pos.pose.orientation.y = 0.0;
    marker_pos.pose.orientation.z = 0.0;
    marker_pos.pose.orientation.w = 1.0;
    marker_pos.scale.x = 0.01;
    marker_pos.scale.y = 0.01;
    marker_pos.scale.z = 0.01;
    marker_pos.color.r = 0.0f;
    marker_pos.color.g = 1.0f;
    marker_pos.color.b = 0.0f;
    marker_pos.color.a = 1.0;
    marker_pos.lifetime = ros::Duration();
    marker_pos_.publish(marker_pos);

    shape = visualization_msgs::Marker::ARROW;
    marker_pip.header.frame_id = "/panda_link0";
    marker_pip.header.stamp = ros::Time::now();
    marker_pip.ns = "orientation";
    marker_pip.id = 0;
    marker_pip.type = shape;
    marker_pip.action = visualization_msgs::Marker::ADD;
    marker_pip.scale.x = 0.005;
    marker_pip.scale.y = 0.001;
    marker_pip.scale.z = 0.001;
    marker_pip.color.r = 1.0f;
    marker_pip.color.g = 0.0f;
    marker_pip.color.b = 0.0f;
    marker_pip.color.a = 1.0;

    marker_pip.points.resize(2);
    
    marker_pip.points[0].x = position[0];
    marker_pip.points[0].y = position[1];
    marker_pip.points[0].z = position[2];

    marker_pip.points[1].x = position[0] + incline_[0]/incline_.norm();
    marker_pip.points[1].y = position[1] + incline_[1]/incline_.norm();
    marker_pip.points[1].z = position[2] + incline_[2]/incline_.norm();

    marker_pip.lifetime = ros::Duration();
    marker_pip_.publish(marker_pip);


    count_markers_ = 0;
  }
  else {
    count_markers_++;
  }
}


}  // namespace franka_force_control

PLUGINLIB_EXPORT_CLASS(franka_force_control::ImpedantExplorer,
                       controller_interface::ControllerBase)
