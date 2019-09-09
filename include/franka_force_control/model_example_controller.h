// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <Eigen/Core>

#include "std_msgs/Float32MultiArray.h"


namespace franka_force_control {

class ModelExampleController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration&) override;

 private:
  franka_hw::FrankaStateInterface* franka_state_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
  franka_hw::FrankaModelInterface* model_interface_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  franka_hw::TriggerRate rate_trigger_{10.};

  // std::array<double, 7>  tau_ext_initial_;
  // std::array<double, 6>  ext_force_initial_;
  // std::array<double, 7>  tau_error_;

  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 6, 1> k_ext_force_initial_, o_ext_force_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;

  ros::Publisher tau_ext_pub_, tau_f_ext_full_pub_, tau_f_ext_pub_;


};

}  // namespace franka_force_control
