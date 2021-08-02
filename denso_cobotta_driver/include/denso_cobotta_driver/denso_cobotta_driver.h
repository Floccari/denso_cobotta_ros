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

#ifndef DENSO_COBOTTA_DRIVER_H
#define DENSO_COBOTTA_DRIVER_H

// C++ standard
#include <iostream>
#include <string>
#include <cerrno>
#include <cstring>
#include <array>
#include <mutex>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "denso_cobotta_interfaces/srv/get_brake_state.hpp"
#include "denso_cobotta_interfaces/srv/get_motor_state.hpp"
#include "denso_cobotta_interfaces/srv/set_brake_state.hpp"
#include "denso_cobotta_interfaces/srv/set_motor_state.hpp"
#include "denso_cobotta_interfaces/srv/set_led_state.hpp"
#include "denso_cobotta_interfaces/srv/clear_error.hpp"
#include "denso_cobotta_interfaces/srv/clear_robot_error.hpp"
#include "denso_cobotta_interfaces/srv/clear_safe_state.hpp"
#include "denso_cobotta_interfaces/msg/robot_state.hpp"
#include "denso_cobotta_interfaces/msg/safe_state.hpp"
#include "denso_cobotta_interfaces/srv/exec_calset.hpp"

// COBOTTA device driver
#include <fcntl.h>
#include <sys/ioctl.h>
#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/cobotta.h"
#include "denso_cobotta_lib/publish_info.h"

namespace denso_cobotta_driver
{
using namespace cobotta_common;
using namespace denso_cobotta_lib;

/**
 * ROS node: DensoCobottaDriver
 */
class DensoCobottaDriver
{
public:
  DensoCobottaDriver();
  virtual ~DensoCobottaDriver() = default;

  bool initialize(rclcpp::Node::SharedPtr nh);
  void start();
  void stop();
  void terminate();
  void update();
  void publish(const bool sync, denso_cobotta_lib::cobotta::PublishInfo pi);

  // Service callback functions.
  bool setMotorStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::SetMotorState::Request> req,
		       std::shared_ptr<denso_cobotta_interfaces::srv::SetMotorState::Response> res);
  bool getMotorStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::GetMotorState::Request> req,
		       std::shared_ptr<denso_cobotta_interfaces::srv::GetMotorState::Response> res);
  bool setBrakeStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::SetBrakeState::Request> req,
		       std::shared_ptr<denso_cobotta_interfaces::srv::SetBrakeState::Response> res);
  bool getBrakeStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::GetBrakeState::Request> req,
		       std::shared_ptr<denso_cobotta_interfaces::srv::GetBrakeState::Response> res);
  bool execCalsetSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ExecCalset::Request> req,
		    std::shared_ptr<denso_cobotta_interfaces::srv::ExecCalset::Response> res);
  bool clearErrorSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ClearError::Request> req,
		    std::shared_ptr<denso_cobotta_interfaces::srv::ClearError::Response> res);
  bool clearRobotErrorSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ClearRobotError::Request> req,
			 std::shared_ptr<denso_cobotta_interfaces::srv::ClearRobotError::Response> res);
  bool clearSafeStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ClearSafeState::Request> req,
			std::shared_ptr<denso_cobotta_interfaces::srv::ClearSafeState::Response> res);
  bool setLedStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::SetLEDState::Request> req,
		     std::shared_ptr<denso_cobotta_interfaces::srv::SetLEDState::Response> res);

  // Subscriber callback functions.
  void miniIoOutputCallback(const std_msgs::msg::UInt16::SharedPtr msg);

private:
  struct MoveParam
  {
    std::array<double, CONTROL_JOINT_MAX> target_position;
    std::array<double, CONTROL_JOINT_MAX> max_velocity;
    std::array<double, CONTROL_JOINT_MAX> max_acceleration;
    int16_t current_limit[JOINT_MAX];
    int16_t current_offset[JOINT_MAX];
  };
  bool activateCalset(rclcpp::Node::SharedPtr nh);
  bool loadJointLimitsParams(rclcpp::Node::SharedPtr nh);

  void dequeueDriver(long arm_no, int count, bool sync = false);
  void dequeueSafetyMcu(int count, bool sync = false);
  void putRosLog(const char* tag, uint32_t main_code, uint32_t sub_code);
  const std::shared_ptr<cobotta::Cobotta>& getCobotta() const;

  bool setDeviationParameters(const std::array<uint16_t, JOINT_MAX + 1>& values);
  bool recvPulse(long arm_no, std::array<int32_t, JOINT_MAX>& pulse);
  bool sineMove(const MoveParam& move_param);
  bool calculateVelocity(const MoveParam& move_param, const std::array<double, CONTROL_JOINT_MAX>& rotation_angle,
                         std::array<double, CONTROL_JOINT_MAX>& velocity, int32_t& max_count);
  bool sendServoUpdateData(const SRV_COMM_SEND& send_data);

  bool isForceClearFlag() const;
  void setForceClearFlag(bool);

  // Service server
  rclcpp::Service<denso_cobotta_interfaces::srv::SetMotorState>::SharedPtr sv_set_motor_;
  rclcpp::Service<denso_cobotta_interfaces::srv::GetMotorState>::SharedPtr sv_get_motor_;
  rclcpp::Service<denso_cobotta_interfaces::srv::ClearError>::SharedPtr sv_clear_error_;
  rclcpp::Service<denso_cobotta_interfaces::srv::ClearRobotError>::SharedPtr sv_clear_robot_error_;
  rclcpp::Service<denso_cobotta_interfaces::srv::ClearSafeState>::SharedPtr sv_clear_safe_state_;
  rclcpp::Service<denso_cobotta_interfaces::srv::SetBrakeState>::SharedPtr sv_set_brake_;
  rclcpp::Service<denso_cobotta_interfaces::srv::GetBrakeState>::SharedPtr sv_get_brake_;
  rclcpp::Service<denso_cobotta_interfaces::srv::ExecCalset>::SharedPtr sv_exec_calset_;
  rclcpp::Service<denso_cobotta_interfaces::srv::SetLEDState>::SharedPtr sv_set_led_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_function_button_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_plus_button_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_minus_button_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_mini_io_input_;
  rclcpp::Publisher<denso_cobotta_interfaces::msg::RobotState>::SharedPtr pub_robot_state_;
  rclcpp::Publisher<denso_cobotta_interfaces::msg::SafeState>::SharedPtr pub_safe_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_gripper_state_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_mini_io_output_;

  // cobotta
  std::shared_ptr<cobotta::Cobotta> cobotta_;

  std_msgs::msg::Bool function_button_state_;
  std_msgs::msg::Bool plus_button_state_;
  std_msgs::msg::Bool minus_button_state_;
  std_msgs::msg::UInt16 mini_io_state_;

  bool force_clear_flag_ = false;

  std::array<double, CONTROL_JOINT_MAX> max_acceleration_;
  std::array<double, CONTROL_JOINT_MAX> max_velocity_;
  // Rang value
  std::vector<double> rang_value_;
};
}  // namespace denso_cobotta_driver

#endif  // DENSO_COBOTTA_DRIVER_H
