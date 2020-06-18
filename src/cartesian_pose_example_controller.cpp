// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_force_control/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_force_control {

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }
 
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_force_control move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        // return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  sub_pose_ref_ = node_handle.subscribe("/path_ref", 20, &CartesianPoseExampleController::PathRefCallback, this);

  // sub_pose_ref_ = node_handle.subscribe("/pose_ref", 20, &CartesianPoseExampleController::PoseRefCallback, this);
  // sub_force_ref_ = node_handle.subscribe("/force_ref", 20, &CartesianPoseExampleController::ForceRefCallback, this);

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  force_d_target_.setZero();
  torque_d_target_.setZero();
  
  sub_bool_ = node_handle.subscribe("/do_it", 20, &CartesianPoseExampleController::BoolCallback, this);
  do_it = false;

  return true;
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  new_T_target_ = initial_pose_;
  elapsed_time_ = ros::Duration(0.0);


  franka::RobotState initial_state = cartesian_pose_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  std::cout << position_d_ << std::endl;
  // std::cout << orientation_d_ << std::endl;

  Ge_.reset();
  Ge_.setNumerator(1.0, 0.0, 0.0);
  Ge_.setDenominator(K_, B_, M_);
  Ge_.c2d(samplingTime_, "zoh");
         
  Gxr_.reset();
  Gxr_.setNumerator(K_, 0.0, 0.0);
  Gxr_.setDenominator(K_, B_, M_);
  Gxr_.c2d(samplingTime_, "zoh"); 


}

void CartesianPoseExampleController::update(const ros::Time&, const ros::Duration& period) {

  std::array<double, 16> pose_ref_prev = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  std::array<double, 16> pose_ref;
  pose_ref = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  if (arrived == true) {
    pose_ref = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  }
  else {

    if ((counter == 0) && (received_ == true)) {
    // int i = i_point_;
      Eigen::Vector3d dummy_pos;
      dummy_pos << msg_.points[i_point_].positions[1], msg_.points[i_point_].positions[2], msg_.points[i_point_].positions[3];
      pos_d_target_ = dummy_pos;
      pos_d_target_[0] -= 0.1;
      Eigen::Vector3d dummy_ori;
      dummy_ori << msg_.points[i_point_].positions[4], msg_.points[i_point_].positions[5], msg_.points[i_point_].positions[6];
      ori_d_target_ = dummy_ori;

      Eigen::Vector3d ox, oy, oz;
      ox = ori_d_target_;
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

    }
    else {
      if (received_ == true) {
        counter--;

      }
      // std::cout << msg_.points.size() << std::endl;
    }



    // std::array<double, 16> pose_ref;

    for (int i = 0; i < 16; i++){
      pose_ref[i] = filter_params_ * new_T_target_[i] + (1.0 - filter_params_) * pose_ref_prev[i];
    }
  // pose_ref = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;

  }

  Eigen::Map<Eigen::Matrix<double, 4, 4>> pose_ref_m(pose_ref.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4>> pose_ref_prev_m(pose_ref_prev.data());
  // std::cout << (pose_ref_m - pose_ref_prev_m).block(0,3,3,1).transpose() << std::endl;
  // std::cout << pose_ref_prev_m << std::endl;
  // std::cout << "+++++++++++" << std::endl;

  cartesian_pose_handle_->setCommand(pose_ref);
}




void CartesianPoseExampleController::PathRefCallback(
    const trajectory_msgs::JointTrajectory& msg) {

  if (received_ == false) {
    std::cout << "got traj msg" << std::endl;
    msg_ = msg;
    i_point_ = 0;
    received_ = true;    
  }

}


void CartesianPoseExampleController::BoolCallback(
    const std_msgs::Bool msg) {

  std::cout << "aaaaaaaa" << std::endl;
  elapsed_time_ = ros::Duration(0.0);
  do_it = true;

}

}  // namespace franka_force_control

PLUGINLIB_EXPORT_CLASS(franka_force_control::CartesianPoseExampleController,
                       controller_interface::ControllerBase)
