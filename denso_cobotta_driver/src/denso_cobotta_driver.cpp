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
#include <algorithm>
#include <iterator>
#include <thread>
#include <stdexcept>
#include <signal.h>
#include <fstream>
#include <unistd.h>
#include <iomanip>

#include <yaml-cpp/yaml.h>
#include "denso_cobotta_driver/denso_cobotta_driver.h"
#include "denso_cobotta_lib/cobotta.h"
#include "denso_cobotta_lib/gripper.h"

void signalHandler(int sig)
{
  int fd;
  fd = open(cobotta_common::PATH_DEVFILE, O_RDWR);
  if (fd)
  {
    denso_cobotta_lib::cobotta::Motor::sendStop(fd);
    close(fd);
  }
  RCLCPP_INFO(rclcpp::get_logger("driver_logger"), "DensoCobotttaDriver has stopped.");
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("denso_cobotta_driver");

  /*
   * Initialize
   */
  denso_cobotta_driver::DensoCobottaDriver driver;
  bool ret = driver.initialize(nh);
  if (!ret)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), "Failed to initialize COBOTTA device driver.");
    return 1;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("driver_logger"), "Finish to initialize COBOTTA device driver.");

  /* for Ctrl+C */
  signal(SIGINT, signalHandler);

  /*
   * Updating state on another thread
   */
  std::thread update_thread([&driver]() {
    rclcpp::Rate rate(1.0 / cobotta_common::DRIVER_UPDATE_PERIOD);
    while (rclcpp::ok())
    {
      driver.update();
      rate.sleep();
    }
  });

  /*
   * main thread
   */
  driver.start();
  rclcpp::spin(nh);
  driver.stop();
  rclcpp::shutdown();
  update_thread.join();
  driver.terminate();

  return 0;
}

namespace denso_cobotta_driver
{
using namespace denso_cobotta_lib::cobotta;
DensoCobottaDriver::DensoCobottaDriver()
{
  cobotta_ = std::make_shared<Cobotta>();
  force_clear_flag_ = false;
}

/**
 * Initialize this node
 * @param nh Node Handle
 * @return true success to initialize
 * @return false failed to initialize
 */
bool DensoCobottaDriver::initialize(rclcpp::Node::SharedPtr nh)
{
  try
  {
    if (!cobotta_->initialize())
      return false;

    cobotta_->getLed()->forceChange(static_cast<uint32_t>(LedColorTable::White));

    // Service server
    sv_set_motor_ = nh->create_service<denso_cobotta_interfaces::srv::SetMotorState>("set_motor_state",
										     std::bind(&DensoCobottaDriver::setMotorStateSv,
											       this,
											       std::placeholders::_1,
											       std::placeholders::_2));
    sv_get_motor_ = nh->create_service<denso_cobotta_interfaces::srv::GetMotorState>("get_motor_state",
										     std::bind(&DensoCobottaDriver::getMotorStateSv,
											       this,
											       std::placeholders::_1,
											       std::placeholders::_2));
    sv_set_brake_ = nh->create_service<denso_cobotta_interfaces::srv::SetBrakeState>("set_brake_state",
										     std::bind(&DensoCobottaDriver::setBrakeStateSv,
										     this,
										     std::placeholders::_1,
										     std::placeholders::_2));
    sv_get_brake_ = nh->create_service<denso_cobotta_interfaces::srv::GetBrakeState>("get_brake_state",
										   std::bind(&DensoCobottaDriver::getBrakeStateSv,
											     this,
											     std::placeholders::_1,
											     std::placeholders::_2));
    if (activateCalset(nh))
    {
	sv_exec_calset_ = nh->create_service<denso_cobotta_interfaces::srv::ExecCalset>("exec_calset",
											std::bind(&DensoCobottaDriver::execCalsetSv,
												  this,
												  std::placeholders::_1,
												  std::placeholders::_2));
    }
    sv_clear_error_ = nh->create_service<denso_cobotta_interfaces::srv::ClearError>("clear_error",
										    std::bind(&DensoCobottaDriver::clearErrorSv,
											      this,
											      std::placeholders::_1,
											      std::placeholders::_2));
    sv_clear_robot_error_ = nh->create_service<denso_cobotta_interfaces::srv::ClearRobotError>("clear_robot_error",
											       std::bind(&DensoCobottaDriver::clearRobotErrorSv,
													 this,
													 std::placeholders::_1,
													 std::placeholders::_2));
    sv_clear_safe_state_ = nh->create_service<denso_cobotta_interfaces::srv::ClearSafeState>("clear_safe_state",
											     std::bind(&DensoCobottaDriver::clearSafeStateSv,
												       this,
												       std::placeholders::_1,
												       std::placeholders::_2));
    sv_set_led_ = nh->create_service<denso_cobotta_interfaces::srv::SetLEDState>("set_LED_state",
										 std::bind(&DensoCobottaDriver::setLedStateSv,
											   this,
											   std::placeholders::_1,
											   std::placeholders::_2));
    
    // Publisher
    pub_function_button_ = nh->create_publisher<std_msgs::msg::Bool>("function_button", 1);
    pub_plus_button_ = nh->create_publisher<std_msgs::msg::Bool>("plus_button", 1);
    pub_minus_button_ = nh->create_publisher<std_msgs::msg::Bool>("minus_button", 1);
    pub_mini_io_input_ = nh->create_publisher<std_msgs::msg::UInt16>("miniIO_input", 1);
    pub_robot_state_ = nh->create_publisher<denso_cobotta_interfaces::msg::RobotState>("robot_state", 64);
    pub_safe_state_ = nh->create_publisher<denso_cobotta_interfaces::msg::SafeState>("safe_state", 64);
    pub_gripper_state_ = nh->create_publisher<std_msgs::msg::Bool>("gripper_state", 1);

    // Subscriber
    sub_mini_io_output_ = nh->create_subscription<std_msgs::msg::UInt16>("miniIO_output", rclcpp::QoS(10),
									 std::bind(&DensoCobottaDriver::miniIoOutputCallback,
										   this,
										   std::placeholders::_1));

    PublishInfo info = this->getCobotta()->update();
    this->publish(false, info);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return false;
  }

  return true;
}

/**
 * Start this node
 */
void DensoCobottaDriver::start()
{
  try
  {
    /* check */
    if (this->getCobotta()->getMotor()->isRunning())
      return;
    if (this->getCobotta()->getSafetyMcu()->isEmergencyButton())
    {
      // Turn OFF Emergency-stop and execute the command clear_safe_state.
      throw CobottaException(0x81400016);
    }
    if (this->getCobotta()->getSafetyMcu()->isProtectiveButton())
    {
      // Turn OFF Protective-stop signal to execute the command.
      throw CobottaException(0x81400019);
    }

    /* start... */
    if (this->isForceClearFlag())
    {
      this->getCobotta()->getSafetyMcu()->forceMoveToStandby();
    }
    if (!this->getCobotta()->getSafetyMcu()->isNormal())
      this->getCobotta()->getSafetyMcu()->moveToNormal();
    if (this->getCobotta()->getDriver()->isError())
      this->getCobotta()->getDriver()->clearError();

    /* motor on */
    this->getCobotta()->getMotor()->start();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return;
  }
}

/**
 *
 */
void DensoCobottaDriver::stop()
{
  try
  {
    cobotta_->getMotor()->stop();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return;
  }
}
/**
 * Terminate this
 */
void DensoCobottaDriver::terminate()
{
  try
  {
    cobotta_->terminate();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return;
  }
}

/**
 * Update my state.
 */
void DensoCobottaDriver::update()
{
  try
  {
    /*
     * update
     */
    PublishInfo pi = this->getCobotta()->update();
    /*
     * Dequeue & Publish
     */
    this->publish(true, pi);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
  }
}

bool DensoCobottaDriver::activateCalset(rclcpp::Node::SharedPtr nh)
{
  bool enable_calset = false;
  if (!nh->get_parameter("enable_calset", enable_calset))
  {
    RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Failed to get param 'enable_calset'.");
    return false;
  }

  if (enable_calset)
  {
    // Load max_acceleration and max_velocity.
    if (!loadJointLimitsParams(nh))
    {
      return false;
    }

    if (nh->get_parameter("rang_value", rang_value_))
    {
      if (rang_value_.size() == CONTROL_JOINT_MAX)
      {
        return true;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Invalid 'rang_value' length.");
      }
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Failed to get param 'rang_value'.");
    }
  }

  return false;
}

bool DensoCobottaDriver::loadJointLimitsParams(rclcpp::Node::SharedPtr nh)
{
  double max_acceleration, max_velocity;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    if (!nh->get_parameter(std::string("joint_limits/joint_") + std::to_string(i + 1) + std::string("/max_acceleration"),
                     max_acceleration))
    {
      RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Failed to load max_acceleration(J%1d).", i + 1);
      return false;
    }
    max_acceleration_[i] = max_acceleration;
  }

  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    if (!nh->get_parameter(std::string("joint_limits/joint_") + std::to_string(i + 1) + std::string("/max_velocity"),
                     max_velocity))
    {
      RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Failed to load max_velocity(J%1d).", i + 1);
      return false;
    }
    max_velocity_[i] = max_velocity;
  }

  return true;
}

/**
 * Publish my state.
 * @param sync true:sync false:async(first)
 */
void DensoCobottaDriver::publish(const bool sync, PublishInfo info)
{
  bool lv4_error = false;
  bool lv5_error = false;
  bool watchdog_error = false;

  try
  {
    /* Driver */
    for (int i = 0; i < ARM_MAX; i++)
    {
      for (int j = 0; j < info.getDriverQueueSize(i); j++)
      {
        struct StateCode sc = this->getCobotta()->getDriver()->dequeue(i);
        std::string tag = "robot#" + std::to_string(i);
        this->putRosLog(tag.c_str(), sc.main_code, sc.sub_code);

	denso_cobotta_interfaces::msg::RobotState pub;
        pub.arm_no = i;
        pub.state_code = sc.main_code;
        pub.state_subcode = sc.sub_code;
        this->pub_robot_state_->publish(pub);

        auto msg = Message::getMessageInfo(sc.main_code);
        if (msg.level >= 4)
          lv4_error = true;
      }
    }
    /* safetyMCU */
    for (int i = 0; i < info.getSafetyMcuQueueSize(); i++)
    {
      struct StateCode sc = this->getCobotta()->getSafetyMcu()->dequeue();
      this->putRosLog("safety", sc.main_code, sc.sub_code);

      denso_cobotta_interfaces::msg::SafeState pub;
      pub.state_code = sc.main_code;
      pub.state_subcode = sc.sub_code;
      this->pub_safe_state_->publish(pub);

      auto msg = Message::getMessageInfo(sc.main_code);
      if (msg.level >= 4)
        lv4_error = true;
      if (msg.level >= 5)
        lv5_error = true;

      /* watchdog timer error */
      if (Message::isWatchdogTimerError(sc.main_code))
        watchdog_error = true;
    }
    /* motor off on Lv4 error */
    if (sync && lv4_error)
      this->getCobotta()->getMotor()->stop();
    /* On async, watchdog timer error */
    if (!sync && !lv5_error && watchdog_error)
      this->setForceClearFlag(true);

    /* other */
    std_msgs::msg::Bool function_button_state;
    std_msgs::msg::Bool plus_button_state;
    std_msgs::msg::Bool minus_button_state;
    std_msgs::msg::UInt16 mini_io_input;
    std_msgs::msg::Bool gripper_state;

    function_button_state.data = info.isFunctionButton();
    this->pub_function_button_->publish(function_button_state);
    plus_button_state.data = info.isPlusButton();
    this->pub_plus_button_->publish(plus_button_state);
    minus_button_state.data = info.isMinusButton();
    this->pub_minus_button_->publish(minus_button_state);
    mini_io_input.data = info.getMiniIo();
    this->pub_mini_io_input_->publish(mini_io_input);
    gripper_state.data = info.isGripperState();
    this->pub_gripper_state_->publish(gripper_state);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return;
  }
}
/**
 * Output ROS_CONSOLE
 * @param tag
 * @param main_code
 * @param sub_code
 */
void DensoCobottaDriver::putRosLog(const char* tag, uint32_t main_code, uint32_t sub_code)
{
  cobotta::Message message(main_code, sub_code);
  std::stringstream ss;
  ss << "[Lv" << message.getErrorLevel();
  ss << ":" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << main_code;
  ss << "/" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << sub_code;
  ss << "] ";
  ss << "<" << tag << "> ";
  ss << message.getMessage();
  switch (message.getErrorLevel())
  {
    case 0:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("driver_logger"), ss.str());
      break;
    case 1:
      RCLCPP_WARN_STREAM(rclcpp::get_logger("driver_logger"), ss.str());
      break;
    case 2:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), ss.str());
      break;
    case 3:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), ss.str());
      break;
    case 4:
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), ss.str());
      break;
    default:  // Level-5
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("driver_logger"), ss.str());
      break;
  }
}

/**
 * Make motor to start or stop.
 * $ rosservice call /cobotta/set_motor_state "{true}"
 * @param req true:motor start false:motor stop
 * @param res true:success false:failure
 * @return true
 */
bool DensoCobottaDriver::setMotorStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::SetMotorState::Request> req,
					 std::shared_ptr<denso_cobotta_interfaces::srv::SetMotorState::Response> res)
{
  res->success = true;
  try
  {
    if (req->state)
    {
      cobotta_->getMotor()->start();
    }
    else
    {
      cobotta_->getMotor()->stop();
    }
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}
/**
 * Get the state of motor.
 * rosservice call /cobotta/get_motor_state
 * @param res true:motor running false:motor stop
 * @return true
 */
bool DensoCobottaDriver::getMotorStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::GetMotorState::Request> req,
					 std::shared_ptr<denso_cobotta_interfaces::srv::GetMotorState::Response> res)
{
  bool motor_on;
  res->success = true;
  res->state = true;

  try
  {
    res->state = cobotta_->getMotor()->isRunning();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Make brakes to lock or unlock.
 * $ rosservice call /cobotta/set_brake_state
 * @param req "{state: [bool, bool, bool. bool, bool, bool]}" true:lock false:unlock
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::setBrakeStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::SetBrakeState::Request> req,
					 std::shared_ptr<denso_cobotta_interfaces::srv::SetBrakeState::Response> res)
{
  res->success = true;

  if (req->state.size() != CONTROL_JOINT_MAX)
  {
    RCLCPP_ERROR(rclcpp::get_logger("driver_logger"), "Failed to run 'set_brake_state'. "
              "The nubmer of joint is invalid. (size=%ld)",
              req->state.size());
    res->success = false;
    return true;
  }

  std::array<int, JOINT_MAX> state;
  state.fill(SRV_BRAKE_NONE);
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    if (req->state[i])
    {
      state[i] = SRV_BRAKE_LOCK;
    }
    else
    {
      state[i] = SRV_BRAKE_RELEASE;
    }
  }

  try
  {
    cobotta_->getBrake()->change(0, state);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Get the state of brakes.
 * $ rosservice call /cobotta/get_brake_state
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::getBrakeStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::GetBrakeState::Request> req,
					 std::shared_ptr<denso_cobotta_interfaces::srv::GetBrakeState::Response> res)
{
  res->success = true;

  try
  {
    auto state = cobotta_->getBrake()->getArmState(0);
    res->state.resize(CONTROL_JOINT_MAX);
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      res->state[i] = state[i] == SRV_BRAKE_LOCK ? true : false;
    }
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Save the parameters used for improving accuracy of the pose.
 *
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::execCalsetSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ExecCalset::Request> req,
				      std::shared_ptr<denso_cobotta_interfaces::srv::ExecCalset::Response> res)
{
  res->success = true;
  rclcpp::Rate rate(std::chrono::milliseconds(20));
  static constexpr double delta_degree = 15;

  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    double tmp_degree = std::abs(cobotta_common::CALSET_POSE[i] - rang_value_[i]);
    if (tmp_degree > delta_degree)
    {
      RCLCPP_ERROR(rclcpp::get_logger("driver_logger"), "Rang value differs greatly from mechanical end(J%1d).", i + 1);
      res->success = false;
      return true;
    }
  }

  auto motor = [&](bool on) {
    denso_cobotta_interfaces::srv::SetMotorState::Request motor_req;
    denso_cobotta_interfaces::srv::SetMotorState::Response motor_res;
    motor_req.state = on;
    auto motor_req_ptr = std::make_shared<denso_cobotta_interfaces::srv::SetMotorState::Request>(motor_req);
    auto motor_res_ptr = std::make_shared<denso_cobotta_interfaces::srv::SetMotorState::Response>(motor_res);
    setMotorStateSv(motor_req_ptr, motor_res_ptr);
    return (bool)(motor_res.success);
  };

  auto failed_process = [&]() {
    res->success = false;
    motor(false);
    rate.sleep();  // Wait for stopping.
    setDeviationParameters(POSITION_DEVIATION_PARAMS);
  };

  std::vector<int32_t> pulse_offset;
  pulse_offset = { 0, 0, 0, 0, 0, 0 };
  // ++++++++++++++++++++↓Begin AutoCal↓++++++++++++++++++++
  auto deviation_params = POSITION_DEVIATION_PARAMS;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    deviation_params[i] = INT16_MAX;
  }

  motor(false);
  rate.sleep();       // Wait for stopping.
  if (!setDeviationParameters(deviation_params))  // Send parameters.
  {
    failed_process();
    return true;
  }
  rate.sleep();  // Wait for sending parameters.
  if (!motor(true))
  {
    failed_process();
    return true;
  }
  rate.sleep();  // Wait for starting.

  // Move to start position.
  MoveParam move_param;
  auto set_start_move_param = [&]() {
    move_param.max_acceleration = max_acceleration_;
    move_param.max_velocity = max_velocity_;
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      double tmp_degree = cobotta_common::CALSET_POSE[i];
      move_param.target_position[i] = tmp_degree * M_PI / 180.0;
      move_param.current_offset[i] = 0;
      move_param.current_limit[i] = 0x0FFF;
    }
  };
  set_start_move_param();
  if (!sineMove(move_param))
  {
    failed_process();
    return true;
  }
  rclcpp::Rate rate2(std::chrono::milliseconds(100));
  rate2.sleep();  // Wait to finish moving.
  RCLCPP_INFO(rclcpp::get_logger("driver_logger"), "Moving to the mechanical end .... Please wait for about 60 seconds.");
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    double tmp_degree = cobotta_common::CALSET_POSE[i] > 0 ? cobotta_common::CALSET_POSE[i] + delta_degree :
                                                             cobotta_common::CALSET_POSE[i] - delta_degree;
    move_param.max_velocity[i] = 0.02;
    move_param.target_position[i] = tmp_degree * M_PI / 180.0;
    move_param.current_limit[i] = 0x04FF;
  }
  if (!sineMove(move_param))
  {
    failed_process();
    return true;
  }
  rate2.sleep();  // Wait to finish moving.
  // ++++++++++++++++++++↑End AutoCal↑++++++++++++++++++++

  std::array<int32_t, JOINT_MAX> cur_pulse;  // [pulse]
  recvPulse(0, cur_pulse);                   // Get current pulse.
  motor(false);
  rate.sleep();  // Wait for starting.

  // Caluculate pulse offset to adjust current pulse.
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    pulse_offset[i] = ARM_COEFF_OUTPOS_TO_PULSE[i] * rang_value_[i] * M_PI / 180.0 - cur_pulse[i];
    if (std::abs(pulse_offset[i] / ARM_COEFF_OUTPOS_TO_PULSE[i]) > OFFSET_LIMIT[i])
    {
      RCLCPP_ERROR(rclcpp::get_logger("driver_logger"), "Rang value differs greatly from encoder value(J%1d).", i + 1);
      failed_process();
      return true;
    }
  }
  // Save pulse offset.
  try
  {
    // Create params file.
    {
      std::ifstream ifs(cobotta_common::TEMP_PARAMS_PATH);
      if (!ifs)
      {
        std::ofstream ofs(cobotta_common::TEMP_PARAMS_PATH);
      }
    }
    YAML::Node cobotta_params = YAML::LoadFile(cobotta_common::TEMP_PARAMS_PATH);
    cobotta_params["pulse_offset"] = pulse_offset;
    std::ofstream ofs(cobotta_common::TEMP_PARAMS_PATH);
    ofs << cobotta_params << std::endl;
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("driver_logger"), "Failed to save the CALSET data.");
    failed_process();
    return true;
  }

  setDeviationParameters(POSITION_DEVIATION_PARAMS);
  RCLCPP_INFO(rclcpp::get_logger("driver_logger"), "Success to save the CALSET data.");
  // Output the CALSET data message.
  std::string pulse_offset_message;
  pulse_offset_message = "Pulse offset is [";
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    pulse_offset_message += "J" + std::to_string(i + 1) + ": " + std::to_string(pulse_offset[i]);
    if (i != CONTROL_JOINT_MAX - 1)
    {
      pulse_offset_message += ", ";
    }
  }
  pulse_offset_message += "].";
  RCLCPP_INFO_STREAM(rclcpp::get_logger("driver_logger"), pulse_offset_message);

  // Move to the valid pose of Moveit.
  motor(true);
  rate.sleep();  // Wait for starting.
  set_start_move_param();
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    move_param.target_position[i] = cobotta_common::HOME_POSE[i] * M_PI / 180.0;
  }
  if (!sineMove(move_param))
  {
    RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Failed to move home pose.");
  }
  RCLCPP_INFO(rclcpp::get_logger("driver_logger"), "Success to move home pose.");
  return true;
}

/**
 * Set deviation parameters
 *
 * @param values Parameters to send
 * @return true:success false:failure
 */
bool DensoCobottaDriver::setDeviationParameters(const std::array<uint16_t, JOINT_MAX + 1>& values)
{
  bool success = true;
  try
  {
    std::array<uint16_t, JOINT_MAX + 1> recv_values;
    Driver::writeHwAcyclicCommAll(cobotta_->getFd(), POSITION_DEVIATION_ADDRESS | ACYCLIC_WRITE_MASK, values,
                                  recv_values);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    success = false;
  }
  return success;
}

/**
 * Get encoder values
 *
 * @param arm_no
 * @param pulse Encoder values
 * @return true:success false:failure
 */
bool DensoCobottaDriver::recvPulse(long arm_no, std::array<int32_t, JOINT_MAX>& pulse)
{
  SRV_COMM_RECV recv_data{ 0 };

  try
  {
    recv_data = Driver::readHwEncoder(cobotta_->getFd(), arm_no);
  }
  catch (const CobottaException& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return false;
  }

  for (int i = 0; i < pulse.size(); i++)
  {
    pulse[i] = recv_data.encoder[i];
  }
  return true;
}

/**
 * Move robot arm. The speed follows sine curve.
 *
 * @param move_param The parameter of the move.
 * @return true:success false:failure
 */
bool DensoCobottaDriver::sineMove(const MoveParam& move_param)
{
  std::array<int32_t, JOINT_MAX> cur_pulse;             // [pulse]
  std::array<double, CONTROL_JOINT_MAX> start_pos_rad;  // [rad]
  std::array<double, CONTROL_JOINT_MAX> rotation_rad;   // [rad]
  std::array<double, CONTROL_JOINT_MAX> velocity;       // [rad/s]
  int32_t max_count;

  auto is_motor_runnning = [&]() {
    bool is_running = cobotta_->getMotor()->isRunning();
    if (!is_running)
    {
      RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Motor is not running.");
    }
    return is_running;
  };

  // Get current position
  if (is_motor_runnning() && recvPulse(0, cur_pulse))
  {
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      start_pos_rad[i] = cur_pulse[i] / ARM_COEFF_OUTPOS_TO_PULSE[i];
    }
  }
  else
  {
    return false;
  }
  // Caluculate rotation angle.
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    rotation_rad[i] = move_param.target_position[i] - start_pos_rad[i];
  }
  if (!calculateVelocity(move_param, rotation_rad, velocity, max_count))
  {
    return false;
  }

  // Prepare to move.
  SRV_COMM_SEND send_data;
  send_data.arm_no = 0;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 0;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    send_data.current_limit[i] = move_param.current_limit[i];
    send_data.current_offset[i] = move_param.current_offset[i];
  }

  const double move_time = max_count * SERVO_PERIOD;
  for (int count = 0; count <= max_count; count++)
  {
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      // Calculate position.
      send_data.position[i] =
          ARM_COEFF_OUTPOS_TO_PULSE[i] *
          (start_pos_rad[i] + (velocity[i] * move_time / M_PI) * (1 - cos(M_PI * count / (double)max_count)));
    }

    if (count == max_count)
    {
      send_data.stay_here = 1;
    }
    if (!(is_motor_runnning() && sendServoUpdateData(send_data)))
    {
      return false;
    }
  }
  return true;
}

/**
 * Decide max velocity of sine curve to be within acceleration and velocity limit.
 *
 * @param move_param The parameter of the move.
 * @return true:success false:failure
 */
bool DensoCobottaDriver::calculateVelocity(const MoveParam& move_param,
                                           const std::array<double, CONTROL_JOINT_MAX>& rotation_angle,
                                           std::array<double, CONTROL_JOINT_MAX>& velocity, int32_t& max_count)
{
  // Calculate moving time [s]
  double move_time = 0;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    // Calculate velocity
    double tmp_velocity = std::sqrt(move_param.max_acceleration[i] * std::abs(rotation_angle[i]) *
                                    0.5);  // tmp_velocity is the velocity based on move_param.max_acceleration
    velocity[i] = tmp_velocity < move_param.max_velocity[i] ? tmp_velocity :
                                                              move_param.max_velocity[i];  // velocity[i] is lower one.

    double tmp_time = velocity[i] < std::numeric_limits<double>::epsilon() ? 0 : std::abs(rotation_angle[i]) * M_PI *
                                                                                     0.5 / velocity[i];
    // Based on the one with the longest move time.
    move_time = move_time > tmp_time ? move_time : tmp_time;
  }
  if (move_time == 0)
  {
    return false;
  }
  // Calculate move velocity [rad/s]
  max_count = std::ceil(move_time / SERVO_PERIOD);  // round up
  move_time = max_count * SERVO_PERIOD;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    velocity[i] = rotation_angle[i] * M_PI * 0.5 / move_time;
  }
  return true;
}

bool DensoCobottaDriver::sendServoUpdateData(const SRV_COMM_SEND& send_data)
{
  try
  {
    struct DriverCommandInfo info = Driver::writeHwUpdate(cobotta_->getFd(), send_data);
    if (info.result == 0x0F408101)
    {
      // The current number of commands in buffer is 11.
      // To avoid buffer overflow, sleep 8 msec
      rclcpp::Rate rate3(cobotta_common::getPeriodStd());
      rate3.sleep();
    }
    else if (info.result == 0x84400502)
    {
      // buffer full
      RCLCPP_WARN(rclcpp::get_logger("driver_logger"), "Command buffer overflow...");
      rclcpp::Rate rate3(cobotta_common::getPeriodStd() * 2);
      rate3.sleep();
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    return false;
  }
  return true;
}

/**
 * Make robot & safety errors to clear except for fatal errors.
 * $ rosservice call /cobotta/clear_error
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::clearErrorSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ClearError::Request> req,
				      std::shared_ptr<denso_cobotta_interfaces::srv::ClearError::Response> res)
{
  res->success = true;
  try
  {
    if (cobotta_->getMotor()->isRunning())
      return true;

    cobotta_->getDriver()->clearError();
    cobotta_->getSafetyMcu()->moveToNormal();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Make robot errors to clear except for fatal errors.
 * $ rosservice call /cobotta/clear_robot_error
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::clearRobotErrorSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ClearRobotError::Request> req,
					   std::shared_ptr<denso_cobotta_interfaces::srv::ClearRobotError::Response> res)
{
  res->success = true;
  try
  {
    cobotta_->getDriver()->clearError();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Make state of SafetyMCU to clear errors & move normal except for fatal errors.
 * $ rosservice call /cobotta/clear_safe_state
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::clearSafeStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::ClearSafeState::Request> req,
					  std::shared_ptr<denso_cobotta_interfaces::srv::ClearSafeState::Response> res)
{
  res->success = true;
  try
  {
    cobotta_->getSafetyMcu()->moveToNormal();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Make the color of LED to change.
 * $ rosservice call /cobotta/set_LED_state "{red:255, green:255, blue:255, blink_rate:255}"
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::setLedStateSv(const std::shared_ptr<denso_cobotta_interfaces::srv::SetLEDState::Request> req,
				       std::shared_ptr<denso_cobotta_interfaces::srv::SetLEDState::Response> res)
{
  res->success = true;

  try
  {
    res->success = cobotta_->getLed()->change(req->blink_rate, req->red, req->green, req->blue);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res->success = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
    res->success = false;
  }

  return true;
}

/**
 * Make the value of mini-Output to send.
 * $ rostopic pub /cobotta/miniIO_output std_msgs/Uint16 "data: 0"
 * @param res true:success false:failure
 * @return
 */
void DensoCobottaDriver::miniIoOutputCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  try
  {
    cobotta_->getMiniIo()->sendOutputStateValue(msg->data);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("driver_logger"), e.what());
  }
}

const std::shared_ptr<cobotta::Cobotta>& DensoCobottaDriver::getCobotta() const
{
  return cobotta_;
}

bool DensoCobottaDriver::isForceClearFlag() const
{
  return force_clear_flag_;
}

void DensoCobottaDriver::setForceClearFlag(bool forceClearFlag = false)
{
  force_clear_flag_ = forceClearFlag;
}

}  // namespace denso_cobotta_driver
