/*
 * Copyright (C) 2018-2019  DENSO WAVE INCORPORATED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef DENSO_COBOTTA_HW_H
#define DENSO_COBOTTA_HW_H

// C++ standard
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cerrno>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/system_interface.hpp>
//#include <joint_limits_interface/joint_limits_interface.h>
#include "denso_cobotta_interfaces/srv/set_motor_state.h"
#include "denso_cobotta_interfaces/srv/set_led_state.h"
#include "denso_cobotta_interfaces/srv/clear_error.h"
#include "denso_cobotta_interfaces/msg/robot_state.h"

// COBOTTA device driver
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/cobotta_exception.h"

namespace denso_cobotta_control
{
using namespace cobotta_common;

class DensoCobottaHW : public hardware_interface::SystemInterface
{
public:
  DensoCobottaHW();
  virtual ~DensoCobottaHW();

  bool initialize(ros::NodeHandle& nh);
  void checkMotorState(void);

  bool read(ros::Time, ros::Duration);
  bool write(ros::Time, ros::Duration);
  bool isMotorOn();
  bool shouldReset();
  void clearReset();

  static void sendStayHere(int fd);
  int getFd() const;

  // Subscriber callback functions.
  void subRobotState(const denso_cobotta_driver::RobotState& msg);

  // Subscriber
  ros::Subscriber sub_robot_state_;

private:
  bool loadCalsetData();
  bool sendServoUpdateData();
  bool recvEncoderData();

  hardware_interface::JointStateInterface jntStInterface_;
  hardware_interface::PositionJointInterface posJntInterface_;

  double cmd_[CONTROL_JOINT_MAX];
  double pos_[CONTROL_JOINT_MAX];
  double vel_[CONTROL_JOINT_MAX];
  double eff_[CONTROL_JOINT_MAX];
  bool motor_on_;  // true:on false:off
  bool reset_;

  std::vector<int32_t> pulse_offset_;

  IOCTL_DATA_UPDATE upd_;

  int fd_;
};
}  // namespace denso_cobotta_control
#endif  // DENSO_COBOTTA_HW_H
