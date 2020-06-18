// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>


#include <franka_hw/franka_cartesian_command_interface.h>

#include <Eigen/Dense>

#include <franka_force_control/Tf2.h>

namespace franka_force_control {

class CartesianPoseExampleController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_{};
  std::array<double, 16> new_T_target_{};


  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Vector3d pos_d_target_, ori_d_target_; 
  Eigen::Quaterniond orientation_d_target_;

  Eigen::Vector3d force_d_target_, torque_d_target_;

  ros::Subscriber sub_pose_ref_;
  ros::Subscriber sub_force_ref_;

  double filter_params_{0.001};

  ros::Subscriber sub_bool_;
  bool do_it;
  void BoolCallback(const std_msgs::Bool msg);
  void PathRefCallback(const trajectory_msgs::JointTrajectory& msg);
  void ForceRefCallback(const geometry_msgs::WrenchConstPtr& msg);


  trajectory_msgs::JointTrajectory msg_;
  int i_point_{0};
  int counter{0};

  bool received_ = false;

  double samplingTime_{0.001};
  double M_{0.}, B_{0.}, K_{0.};
  Tf2 Ge_, Gxr_;
  
  bool arrived{false};
};

}  // namespace franka_force_control
