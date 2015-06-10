/**
 *
 *  \file
 *  \brief      Class representing Ridgeback hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <boost/assign.hpp>
#include "ridgeback_base/ridgeback_hardware.h"
#include "puma_motor_driver/driver.h"


namespace ridgeback_base
{

RidgebackHardware::RidgebackHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                     puma_motor_driver::Gateway& gateway):
  nh_(nh),
  pnh_(pnh),
  gateway_(gateway)
{
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");

  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }

  drivers_.push_back(puma_motor_driver::Driver(gateway_, 3, "front_left_wheel"));
  drivers_.push_back(puma_motor_driver::Driver(gateway_, 5, "front_right_wheel"));
  drivers_.push_back(puma_motor_driver::Driver(gateway_, 2, "rear_left_wheel"));
  drivers_.push_back(puma_motor_driver::Driver(gateway_, 4, "rear_right_wheel"));

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.clearStatusCache();
    driver.setEncoderCPR(1024);
    driver.setGearRatio(79.0);
    driver.setMode(puma_motor_msgs::Status::MODE_SPEED, -0.1, -0.01, 0.0);
  }
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the motor controller.
 *
 * Called from the controller thread.
 */
void RidgebackHardware::updateJointsFromHardware()
{
  //boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  //if (feedback_msg_ && feedback_msg_lock)
  //{
    for (int i = 0; i < 4; i++)
    {
      joints_[i].position = feedback_msg_->drivers_feedback[i].travel;
      joints_[i].velocity = feedback_msg_->drivers_feedback[i].speed;
      joints_[i].effort = feedback_msg_->drivers_feedback[i].current;
    }
  //}
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void RidgebackHardware::command()
{
  for (int i = 0; i < 4; i++)
  {
    drivers_[i].commandSpeed(joints_[i].velocity_command);
  }
}

}  // namespace ridgeback_base
