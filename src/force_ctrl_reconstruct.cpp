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
#define N_ORDER 3
#define MIN_DIST_ 0.1
#define MAX_HIST_LEN_ 15
#define MIN_DIST_THRES_ 0.01

namespace franka_force_control {

bool ForceCtrlReconstruct::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceCtrlReconstruct: Assuming SOFT gripper!");
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
  if (ros::param::has("/imp_scale_m_")) {
    ros::param::get("/imp_scale_m_", imp_scale_m_);
    target_imp_scale_m_ = imp_scale_m_;
  }
  if (ros::param::has("/imp_scale_m_i_")) {
    ros::param::get("/imp_scale_m_i_", imp_scale_m_i_);
    target_imp_scale_m_i_ = imp_scale_m_i_;
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
  gripper_type_sub_ = node_handle.subscribe("/gripper_rigid", 1, 
    &ForceCtrlReconstruct::gripper_type_callback, this);


  sub_pose_ref_ = node_handle.subscribe("/path_ref", 20, &ForceCtrlReconstruct::PathRefCallback, this);


  marker_pos_ = node_handle.advertise<visualization_msgs::Marker>("/franka_pos", 1);
  marker_pip_ = node_handle.advertise<visualization_msgs::Marker>("/pipe_ori", 1);

  return true;
}

void ForceCtrlReconstruct::gripper_type_callback(
  const std_msgs::Bool::ConstPtr& msg) {
  gripper_rigid_ = msg->data;
}

void ForceCtrlReconstruct::reset_callback(
  const franka_msgs::ErrorRecoveryActionGoal&  msg) {
  f_err_int_.setZero();
  polyfit_history = 0;
  flag_reset_ = 0;
  moments_integrate_.setZero();
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

  history_pos_.setZero(3,MAX_HIST_LEN_);

  history_pos_.block(0,MAX_HIST_LEN_-1,3,1) = pos_global_prev_;

  incline_.setZero(3,1);
  target_incline_.setZero(3,1);

  target_pos_ref_ = pos_global_prev_.block(0,0,3,1);

  cummulative_dist_.setZero(MAX_HIST_LEN_);
  distances_.setZero(MAX_HIST_LEN_);

  moments_integrate_.setZero(3);


  pos_d_target_ = target_pos_ref_;
  ori_d_target_.setZero();
  new_T_target_ = T_base_arr;
  Eigen::Vector3d pos_d_target_, ori_d_target_; 
  std::array<double, 16> new_T_target_{};

}

void ForceCtrlReconstruct::update(const ros::Time& /*time*/, const ros::Duration& period) {
  
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


  Eigen::VectorXd position(3), incline(3), pos_ref(3);
  Eigen::VectorXd pos_loc(4), pos_glob(4);
  pos_loc[0] = 0;
  pos_loc[1] = -0.03;
  pos_loc[2] = 0.03;
  pos_loc[3] = 1;
  pos_glob = T_base * pos_loc;
  position = pos_glob.block(0,0,3,1);
  pos_ref = position;

  // position << T_base_arr[12], T_base_arr[13], T_base_arr[14];

  if (mode_ == franka::RobotMode::kMove && period.toSec() > 0.) {

  //   if (pipe_dir_freq_ == 0) {
  //     pipe_dir_freq_ = 20;
  //     incline = ForceCtrlReconstruct::directionPrediction(position);
  //     target_incline_ = incline;
  //   }
  //   else {
  //     pipe_dir_freq_ -= 1;
  //   }




    if (received_ == true) {
      if (counter == 0) {
        Eigen::Vector3d dummy_pos;
        dummy_pos << msg_.points[i_point_].positions[1], msg_.points[i_point_].positions[2], msg_.points[i_point_].positions[3];
        pos_d_target_ = dummy_pos;
        pos_d_target_[0] -= 0.1;
        Eigen::Vector3d dummy_ori;
        dummy_ori << msg_.points[i_point_].positions[4], msg_.points[i_point_].positions[5], msg_.points[i_point_].positions[6];
        ori_d_target_ = dummy_ori;

        Eigen::Vector3d ox, oy, oz;
        ox = -ori_d_target_;
        oz << 0., 0., -1;
        oy = ox.cross(oz);
        oz = ox.cross(oy);

        Eigen::Matrix<double, 4, 4> new_T;
        new_T.setZero();
        new_T(3,3) = 1.;
        for (int i = 0; i < 3; i++){
          new_T(i,0) = ox[i];
          new_T(i,1) = oy[i];
          new_T(i,2) = oz[i];

          new_T(i,3) = pos_d_target_[i] - 0.04 * new_T(i,2);
        }
        for(int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            new_T_target_[i*4+j] = new_T(j,i);
          }
        }
        if (i_point_ < msg_.points.size() - 1) {
          i_point_++;
        }
        counter = 100;
        std::cout << i_point_ << std::endl;

        target_incline_ = ox;
        target_pos_ref_ = new_T.block(0,3,3,1);

      }
      else {
        counter--;
      }
    } 

    incline_ = 0.9 * incline_ + 0.1 * target_incline_;
    pos_ref_ = 0.9 * pos_ref_ + 0.1 * target_pos_ref_;

    force_imp = ForceCtrlReconstruct::impedanceOpenLoop(period, force_meas, incline_, pos_ref_);

    force_pid = ForceCtrlReconstruct::PID(force_meas, force_des + force_imp, period);
    // force_pid = ForceCtrlReconstruct::PID(force_meas, force_des , period);


    if (count_markers_ == 100) {
      marker_id_ += 1;
      visualization_msgs::Marker marker_pos, marker_pip;
      uint32_t shape = visualization_msgs::Marker::SPHERE;
      marker_pos.header.frame_id = "/my_frame";
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
      marker_pip.header.frame_id = "/my_frame";
      marker_pip.header.stamp = ros::Time::now();
      marker_pip.ns = "orientation";
      marker_pip.id = 0;
      marker_pip.type = shape;
      marker_pip.action = visualization_msgs::Marker::ADD;
      // marker_pip.pose.position.x = position[0];
      // marker_pip.pose.position.y = position[1];
      // marker_pip.pose.position.z = position[2];
      // marker_pip.pose.orientation.x = 0.0;
      // marker_pip.pose.orientation.y = 0.0;
      // marker_pip.pose.orientation.z = 0.0;
      // marker_pip.pose.orientation.w = 1.0;
      marker_pip.scale.x = 0.005;
      marker_pip.scale.y = 0.001;
      marker_pip.scale.z = 0.001;
      marker_pip.color.r = 1.0f;
      marker_pip.color.g = 0.0f;
      marker_pip.color.b = 0.0f;
      marker_pip.color.a = 1.0;
      // std::cout << "aaaaaaaa11" << std::endl;

      marker_pip.points.resize(2);
      
      marker_pip.points[0].x = position[0];
      marker_pip.points[0].y = position[1];
      marker_pip.points[0].z = position[2];

      marker_pip.points[1].x = position[0] + incline_[0]/incline_.norm();
      marker_pip.points[1].y = position[1] + incline_[1]/incline_.norm();
      marker_pip.points[1].z = position[2] + incline_[2]/incline_.norm();
      
      // std::cout << "aaaaaaaa22" << std::endl;
      marker_pip.lifetime = ros::Duration();
      marker_pip_.publish(marker_pip);


      count_markers_ = 0;
    }
    else {
      count_markers_++;
    }



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

    imp_scale_m_ = target_imp_scale_m_; 
    imp_scale_m_i_ = target_imp_scale_m_i_; 
    

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
              const ros::Duration& period, Eigen::Matrix<double, 6, 1> f_ext, 
              Eigen::VectorXd incline, Eigen::VectorXd pos_ref){


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

  // dXglobal = T_base * dXlocal - posGlobal4;

  dXglobal.block(0,0,3,1) = pos_ref - posGlobal4.block(0,0,3,1);

  Eigen::Matrix<double, 3, 1> velRefLoc, velRefGlob, accRefGlob;
  // double dxAmpl{0.};
  // for (int i = 0; i < 3; i++){
  //   dxAmpl += pow(dXlocal(i), 2);
  //   velRefLoc(i) = dXlocal(i);
  // } 
  // dxAmpl = sqrt(dxAmpl);
  // if (dxAmpl >= 0.1) {
  //   velRefLoc = 0.2 * velRefLoc / dxAmpl;
  // }

  Eigen::Matrix<double, 3, 3> R_base;
  R_base = T_base.block<3,3>(0,0);

  // velRefGlob = R_base * velRefLoc;
  velRefGlob = 0.02 * dXglobal.block(0,0,3,1)/dXglobal.block(0,0,3,1).norm();
  for (int i = 0; i < 3; i++) {
    // velRefGlob[i] = 0.0;
    accRefGlob[i] = 0.;
  }
  

  velGlobal = (posGlobal - pos_global_prev_) / period.toSec();
  accGlobal = (velGlobal - vel_global_prev_) / period.toSec();

  float velAmpl{0.}, impRefAmpl{0.};
  velAmpl = velGlobal.norm();
  impRefAmpl = dXlocal.block(0,0,3,1).norm();
  // for (int i = 0; i < 3; i++) {
  //   velAmpl += pow(velGlobal[i], 2.);
  //   impRefAmpl += pow(dXlocal[i], 2.);
  // }
  // velAmpl = sqrt(velAmpl);
  // impRefAmpl = sqrt(impRefAmpl);

  Eigen::Matrix<double, 3, 1> momentsLoc, moments, moments_wiggle;
  moments.setZero();
  moments_wiggle.setZero();
  if (impRefAmpl >= 0.1) {
    momentsLoc = wiggle(velAmpl);
    moments_wiggle = R_base * momentsLoc;
  }

  force = (imp_m_ / imp_scale_ - 1.0) * f_ext.block<3,1>(0,0) 
              // - imp_m_ * accRefGlob
              - (imp_m_ / imp_scale_) * (imp_d_ * (velGlobal - velRefGlob)
                - imp_k_ * dXglobal.block(0,0,3,1));


  Eigen::VectorXd refDirLoc(3), refDirGlob(3);

  if (velRefGlob.norm() > 0.) {
    refDirLoc << 1.0, 0.0, 0.0;
    refDirLoc /= refDirLoc.norm();
    refDirGlob = R_base * refDirLoc;
    

    moments_integrate_ = moments_integrate_ + (incline-refDirGlob) * period.toSec();

    double integrator_limit_moments = integrator_limit_/3.0;
    for(int i = 0; i < 3; i++) {
      if (moments_integrate_[i] > integrator_limit_moments)
        moments_integrate_[i] = integrator_limit_moments;
      if (moments_integrate_[i] < - integrator_limit_moments)
        moments_integrate_[i] = -integrator_limit_moments;
    }

    moments = imp_scale_m_ * (incline - refDirGlob) + imp_scale_m_i_ * moments_integrate_;     
  }
  else {
    refDirLoc.setZero();
    refDirGlob.setZero();
    moments.setZero();
  }

  force_torque.setZero();

  for(int i = 0; i < 3; i++) {
    force_torque[i] = force[i];
    // if (gripper_rigid_ == false) {
    //   force_torque[i+3] = moments[i] + 0.5 * moments_wiggle[i];
    // }
    // else
    //   force_torque[i+3] = moments[i] + moments_wiggle[i];
  }

  pos_global_prev_ = posGlobal;
  vel_global_prev_ = velGlobal;



  std_msgs::Float32MultiArray array_dxglob, array_posglob;
  for(int i = 0; i < 3; i++) {
    array_dxglob.data.push_back(incline[i] - refDirGlob[i]);
    // array_dxglob.data.push_back(pos_ref[i]);
    array_posglob.data.push_back(pos_ref[i]);
    // array_dxglob.data.push_back(dXglobal[i]);
    // array_dxglob.data.push_back(velGlobal[i]);
    // array_posglob.data.push_back(posGlobal[i]);
  }


  array_dxglob.data.push_back(velAmpl);
  pos_ref_glob_pub.publish(array_dxglob);
  pos_glob_pub.publish(array_posglob);



  return force_torque;

}


Eigen::Matrix<double, 3, 1> ForceCtrlReconstruct::wiggle(float velocity_amplitude) {
  
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

    // // rand_vec *= 0.1;
    // // rand_vec -= wiggle_moments_;
    // rand_vec_ampl = 0.;
    // for(int i = 0; i < 3; i++){
    //   rand_vec[i] = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    //   rand_vec_ampl += pow(rand_vec[i], 2.);
    // }
    // rand_vec_ampl = sqrt(rand_vec_ampl);
    // rand_vec /= rand_vec_ampl;

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

Eigen::VectorXd ForceCtrlReconstruct::directionPrediction(Eigen::VectorXd position) {

  Eigen::VectorXd new_dist_vec;

  new_dist_vec = history_pos_.block(0,MAX_HIST_LEN_-1,3,1) - position;
  double new_dist = new_dist_vec.norm();

  int n = polyfit_history; // number of points
  int len = n - 1; // largest index

  // std::cout << "new dist " << new_dist << std::endl;

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

  // float dist = 0.;
  // float dist_1 = 0.;

  // Eigen::VectorXd cum_dist = Eigen::VectorXd::Zero(n);

  // int i_points = 0;
  // Eigen::VectorXd resultant;
  // while ((fabs(dist) < MIN_DIST_) && (i_points < polyfit_history-1)) {
  //   dist_1 = 0.;
  //   resultant = history_pos_.col(n-i_points) - history_pos_.col(n-(i_points+1));
  //   dist_1 = resultant.norm();
  //   // for (int i = 0; i < 3; i++) {

  //   //   dist_1 += pow(history_pos_(i, n-i_points) - history_pos_(i, n-(i_points+1)), 2);
  //   // }
  //   cum_dist(i_points) = cum_dist(i_points-1) + sqrt(dist_1);
  //   dist += (cum_dist(i_points-1) - cum_dist(i_points));
  //   i_points++;
  // }

  // polyfit_history = i_points + 1;

  bool standing_still = false;

  // if (cummulative_dist_.sum() < MIN_DIST_) {
  //   standing_still = true;
  // }

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
      coeffs.col(i) = ForceCtrlReconstruct::polyfit(timeparam, dimension, N_ORDER);
      incline(i) = - coeffs(2,i);
    }

    if (incline.norm() != 0.)
      incline = incline / incline.norm();

    if (std::isnan(incline.norm())) {
      incline.setZero(3,1);
      // std::cout << history_pos_ << std::endl;
      std::cout << "NAN" << std::endl;
    }

    if (flag_reset_ < 10){
      // int pos;
      // double maxdist;
      // maxdist = (cum_dist.block(1,0,cum_dist.size()-1,1)-cum_dist.block(0,0,cum_dist.size()-1,1)).maxCoeff(&pos);
      // std::cout << "maxdist " << maxdist << " index " << pos << std::endl;
      // // print largest dist & points around
      // std::cout << history_pos_.block(0,MAX_HIST_LEN_-1-pos,3,1).transpose() << std::endl;
      // std::cout << history_pos_.block(0,MAX_HIST_LEN_-1-(pos+1),3,1).transpose() << std::endl;
      // std::cout << history_pos_.block(0,MAX_HIST_LEN_-1-(pos+2),3,1).transpose() << std::endl;
      // std::cout << incline.transpose() << std::endl;
      // std::cout << incline.norm() * 100. << std::endl;
      // std::cout << history_pos_.block(0,MAX_HIST_LEN_-2,3,1).transpose() << std::endl;
      // std::cout << history_pos_.block(0,MAX_HIST_LEN_-1,3,1).transpose() << std::endl;
      // std::cout << "======================" << std::endl;
      flag_reset_ += 1;

    }
  }

  return incline;

}



void ForceCtrlReconstruct::PathRefCallback(
    const trajectory_msgs::JointTrajectory& msg) {

  if (received_ == false) {
    std::cout << "got traj msg" << std::endl;
    msg_ = msg;
    i_point_ = 0;
    received_ = true;    
  }

}


Eigen::VectorXd ForceCtrlReconstruct::polyfit(Eigen::VectorXd t, Eigen::VectorXd y,int order) 
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


}  // namespace franka_force_control

PLUGINLIB_EXPORT_CLASS(franka_force_control::ForceCtrlReconstruct,
                       controller_interface::ControllerBase)
