/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#include "vesc_hw_interface/vesc_servo_controller.h"

namespace vesc_hw_interface
{
VescServoController::VescServoController()
  : gear_ratio_(1.0),
  torque_const_(1.0),
  num_motor_pole_pairs_(1)
{
}

VescServoController::~VescServoController()
{
  interface_ptr_->setDutyCycle(0.0);
}

void VescServoController::init(ros::NodeHandle nh, VescInterface* interface_ptr)
{
  // initializes members
  if (interface_ptr == NULL)
  {
    ros::shutdown();
  }
  else
  {
    interface_ptr_ = interface_ptr;
  }

  calibration_flag_ = true;
  zero_position_ = 0.0;
  error_integ_ = 0.0;

  // reads parameters
  nh.param("servo/Kp", Kp_, 50.0);
  nh.param("servo/Ki", Ki_, 0.0);
  nh.param("servo/Kd", Kd_, 1.0);
  nh.param("servo/control_rate", control_rate_, 100.0);
  control_period_ = 1.0 / control_rate_;
  nh.param("servo/calibration_current", calibration_current_, 6.0);
  nh.param("servo/calibration_duty", calibration_duty_, 0.1);
  nh.param<std::string>("servo/calibration_mode", calibration_mode_, "current");
  nh.param("servo/calibration_position", calibration_position_, 0.0);
  nh.param("servo/speed_limit", speed_limit_, 1.0);

  // shows parameters
  ROS_INFO("[Servo Gains] P: %f, I: %f, D: %f", Kp_, Ki_, Kd_);
  if (calibration_mode_ == CURRENT)
  {
    ROS_INFO("[Servo Calibration] Mode: %s, value: %f", CURRENT.data(), calibration_current_);
  }
  else if (calibration_mode_ == DUTY)
  {
    ROS_INFO("[Servo Calibration] Mode: %s, value: %f", DUTY.data(), calibration_duty_);
  }
  else
  {
    ROS_ERROR("[Servo Calibration] Invalid mode");
  }
  // Create timer callback for PID servo control
  control_timer_ = nh.createTimer(ros::Duration(control_period_), &VescServoController::controlTimerCallback, this);
  return;
}

void VescServoController::control(const double position_reference, const double position_current)
{
  // executes caribration
  if (calibration_flag_)
  {
    calibrate(position_current);
    // initializes/resets control variables
    time_previous_ = ros::Time::now();
    position_sens_previous_ = position_current;
    position_reference_ = calibration_position_;
    position_reference_previous_ = calibration_position_;
    error_previous_ = 0.0;
    return;
  }

  const ros::Time time_current = ros::Time::now();
  const double dt = control_period_;//(time_current - time_previous_).toSec();
  // calculates PD control
  const double error_current = position_reference - position_current;
  const double u_p = Kp_ * error_current + Kd_;
  // const double u_pd = u_p + Kd_ * (smoothDifferentiate(error_current, error_previous_, dt));
  const double u_pd = Kp_ * error_current + Kd_ * (error_current - error_previous_) / dt;

  double u = 0.0;

  // calculates I control if PD input is not saturated
  if (isSaturated(u_pd))
  {
    u = saturate(u_pd);
  }
  else
  {
    double error_integ_new = error_integ_ + (error_current + error_previous_) / 2.0 * dt;
    const double u_pid = u_pd + Ki_ * error_integ_new;

    // not use I control if PID input is saturated
    // since error integration causes bugs
    if (isSaturated(u_pid))
    {
      u = saturate(u_pid);
    }
    else
    {
      u = u_pid;
      error_integ_ = error_integ_new;
    }
  }

  // updates previous data
  error_previous_ = error_current;
  time_previous_ = time_current;
  position_sens_previous_ = position_current;
  position_reference_previous_ = position_reference;

  // command duty
  interface_ptr_->setDutyCycle(u);
  return;
}

void VescServoController::setTargetPosition(const double position_reference)
{
  // position_reference_ = position_reference;
  position_target_ = position_reference;
}

void VescServoController::setGearRatio(const double gear_ratio)
{
  gear_ratio_ = gear_ratio;
  ROS_INFO("[VescServoController]Gear ratio is set to %f", gear_ratio_);
}

void VescServoController::setTorqueConst(const double torque_const)
{
  torque_const_ = torque_const;
  ROS_INFO("[VescServoController]Torque constant is set to %f", torque_const_);
}

void VescServoController::setMotorPolePairs(const int motor_pole_pairs)
{
  num_motor_pole_pairs_ = motor_pole_pairs;
  ROS_INFO("[VescServoController]The number of motor pole pairs is set to %d", num_motor_pole_pairs_);
}


double VescServoController::getZeroPosition() const
{
  return zero_position_;
}

double VescServoController::getPositionSens(void)
{
  if (calibration_flag_)
  {
    return calibration_position_;
  }
  return position_sens_;
}

double VescServoController::getVelocitySens(void)
{
  return velocity_sens_;
}

double VescServoController::getEffortSens(void)
{
  return effort_sens_;
}

void VescServoController::executeCalibration()
{
  calibration_flag_ = true;
  return;
}

bool VescServoController::calibrate(const double position_current)
{
  static double position_previous;
  static uint16_t step = 0;

  // sends a command for calibration
  if (calibration_mode_ == CURRENT)
  {
    interface_ptr_->setCurrent(calibration_current_);
  }
  else if (calibration_mode_ == DUTY)
  {
    interface_ptr_->setDutyCycle(calibration_duty_);
  }
  else
  {
    ROS_ERROR("Please set the calibration mode surely");
    return false;
  }

  step++;

  if (step % 20 == 0)
  {
    if(position_current == position_previous)
    {
      // finishes calibrating
      step = 0;
      zero_position_ = position_current - calibration_position_;
      position_sens_ = calibration_position_;
      position_sens_previous_ = calibration_position_;
      position_target_ = calibration_position_;
      position_reference_ = calibration_position_;
      position_reference_previous_ = calibration_position_;
      ROS_INFO("Calibration Finished");
      calibration_flag_ = false;
      return true;
    }
    else
    {
      position_previous = position_current;
      return false;
    }
  }
  else
  {
    // continues calibration
    return false;
  }
}

bool VescServoController::isSaturated(const double arg) const
{
  if (std::abs(arg) > 1.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

double VescServoController::saturate(const double arg) const
{
  if (arg > 1.0)
  {
    return 1.0;
  }
  else if (arg < -1.0)
  {
    return -1.0;
  }
  else
  {
    return arg;
  }
}

void VescServoController::updateSpeedLimitedPositionReference(void)
{
  if( position_target_ > (position_reference_previous_ + speed_limit_ * control_period_) )
  {
    position_reference_ = position_reference_previous_ + speed_limit_ * control_period_;
  }
  else if(position_target_ < (position_reference_previous_ - speed_limit_ * control_period_) )
  {
    position_reference_ = position_reference_previous_ - speed_limit_ * control_period_;
  }
  else
  {
    position_reference_ = position_target_;
  }
}

void VescServoController::controlTimerCallback(const ros::TimerEvent& e)
{
  updateSpeedLimitedPositionReference();
  // ROS_INFO("[VescServoController]Target %f, Ref %f, Sens %f", position_target_, position_reference_, position_sens_);
  // position_reference_ = position_target_;
  control(position_reference_, position_sens_);
  interface_ptr_->requestState();
}

void VescServoController::packetCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_motor_pole_pairs_);
    const double position_pulse = values->getPosition();

    // 3.0 represents the number of hall sensors
    position_sens_ = position_pulse / num_motor_pole_pairs_ / 3.0 * gear_ratio_ -
                getZeroPosition();  // unit: rad or m

    velocity_sens_ = velocity_rpm / 60.0 * 2.0 * M_PI * gear_ratio_;  // unit: rad/s or m/s
    effort_sens_ = current * torque_const_ / gear_ratio_;             // unit: Nm or N
  }
  return;
}


}  // namespace vesc_hw_interface
