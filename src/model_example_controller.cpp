// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_force_control/model_example_controller.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace franka_force_control {

bool ModelExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("ModelExampleController: Could not get Franka state interface from hardware");
    return false;
  }
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ModelExampleController: Could not read parameter arm_id");
    return false;
  }
  model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM("ModelExampleController: Error getting model interface from hardware");
    return false;
  }

  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ModelExampleController: Exception getting franka state handle: " << ex.what());
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface_->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ModelExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  tau_ext_pub_ = node_handle.advertise<std_msgs::Float32MultiArray>("/tau_ext", 100);
  tau_f_ext_full_pub_ = node_handle.advertise<std_msgs::Float32MultiArray>("/tau_f_ext_full", 100);
  tau_f_ext_pub_ = node_handle.advertise<std_msgs::Float32MultiArray>("/tau_f_ext", 100);



  return true;
}


void ModelExampleController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = franka_state_handle_->getRobotState();
  
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  
  Eigen::Map<Eigen::Matrix<double, 6, 1>> ext_force_k(robot_state.K_F_ext_hat_K.data());

  Eigen::Map<Eigen::Matrix<double, 6, 1>> ext_force_o(robot_state.O_F_ext_hat_K.data());


  k_ext_force_initial_ = ext_force_k;
  o_ext_force_initial_ = ext_force_o;
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();

}


void ModelExampleController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  if (rate_trigger_()) {
    franka::RobotState robot_state = franka_state_handle_->getRobotState();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    std::array<double, 7> gravity_array = model_handle_->getGravity();
    std::array<double, 16> pose = model_handle_->getPose(franka::Frame::kJoint4);
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> ext_force_o(robot_state.O_F_ext_hat_K.data());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> ext_force_k(robot_state.K_F_ext_hat_K.data());


    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(
        robot_state.tau_J_d.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

    Eigen::VectorXd tau_d(7), gravity_force(6), tau_cmd(7), tau_ext(7), tau_d_full(7), tau_k(7);


    tau_ext = tau_measured - gravity - tau_ext_initial_ ;
    tau_d_full << jacobian.transpose() * ext_force_o;
    tau_d << jacobian.transpose() * (ext_force_o - o_ext_force_initial_) ;

    // std::array<double, 6> o_f_ext = robot_state.O_F_ext_hat_K;
    // std::array<double, 6> k_f_ext = robot_state.K_F_ext_hat_K;

    std_msgs::Float32MultiArray array_ext, array_d_full, array_d;
    for(int i = 0; i < 7; i++) {
      array_ext.data.push_back(tau_ext[i]);
      array_d_full.data.push_back(tau_d_full[i]);
      array_d.data.push_back(tau_d[i]);
    }
    tau_ext_pub_.publish(array_ext);
    tau_f_ext_full_pub_.publish(array_d_full);
    tau_f_ext_pub_.publish(array_d);

    ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("ofext :" << (ext_force_o - o_ext_force_initial_).transpose());
    // ROS_INFO_STREAM("kfext :" << (ext_force_k - k_ext_force_initial_).transpose());
    // ROS_INFO_STREAM("calc_f_ext :" << (calc_f_ext).transpose());
    ROS_INFO_STREAM("tau_ext :" << (tau_ext).transpose());
    ROS_INFO_STREAM("tau_d_full :" << (tau_d_full).transpose());
    ROS_INFO_STREAM("tau_d :" << (tau_d).transpose());

  }
}

}  // namespace franka_force_control

PLUGINLIB_EXPORT_CLASS(franka_force_control::ModelExampleController,
                       controller_interface::ControllerBase)
