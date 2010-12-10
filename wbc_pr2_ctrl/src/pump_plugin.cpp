/*
 * PR2 controller plugin for Stanford-WBC http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file pump_plugin.cpp
   \brief Plugin that pumps data between PR2 (RT controllers) and WBC (non-RT servo).
   \author Roland Philippsen
*/

#include <wbc_pr2_ctrl/mq_robot_api.h>
#include <pr2_controller_interface/controller.h>
#include <wbc_pr2_ctrl/PumpDebugState.h>
#include <wbc_pr2_ctrl/PumpDebugCommand.h>
#include <pluginlib/class_list_macros.h>
#include <jspace/tao_util.hpp>
#include <wbc_urdf/Model.hpp>
#include <ros/console.h>

using namespace std;


// Number of samples in the duty cycle array. In order to find out how
// often the pump is actually getting something from the controller,
// these samples are averaged and sent as part of the robot state and
// debug message.
static size_t const N_DUTY_CYCLE(10);

// Number of samples in the command history array. In order to
// determine how precisely the motors are tracking the desired
// command, we compare the measured effort with the command
// torque... but because there is some amount of delay in the system,
// we're going to be optimistic and search the minimum delta over the
// last N commands.
static size_t const N_COMMAND_HISTORY(10);


class PumpPlugin
  : public pr2_controller_interface::Controller
{
public:
  PumpPlugin();
  virtual ~PumpPlugin();
  
  virtual void update(void);
  virtual bool init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle &nn);
  
  std::vector<pr2_mechanism_model::JointState *> controlled_joint_;
  jspace::ros::Model model_;
  size_t ndof_;
  wbc_pr2_ctrl::MQRobotAPI * mq_robot_api_;
  bool ready_to_send_;
  bool com_enabled_;
  bool mq_blocking_;
  
  int duty_cycle_array_[N_DUTY_CYCLE];
  double * command_history_;	// N_COMMAND_HISTORY * ndof
  int tick_;
  
  ros::Publisher debug_state_pub_;
  ros::Publisher debug_command_pub_;
  wbc_pr2_ctrl::PumpDebugState debug_state_msg_;
  wbc_pr2_ctrl::PumpDebugCommand debug_command_msg_;
  ros::WallTime debug_publish_time_;
  ros::WallDuration debug_publish_delay_;
};


PumpPlugin::
PumpPlugin()
  : model_("/wbc_pr2_ctrl/"),
    ndof_(0),
    mq_robot_api_(0),
    ready_to_send_(true),
    command_history_(0),
    tick_(0),
    debug_publish_delay_(0.05)	// XXXX hardcoded 20Hz
{
  memset(duty_cycle_array_, 0, sizeof(duty_cycle_array_));
}


PumpPlugin::
~PumpPlugin()
{
  delete mq_robot_api_;
  delete[] command_history_;
}


// What we are really interested in is the relative error between the
// desired and the actual torque. However, let's try to avoid
// divisions by small numbers, shall we?
static inline double compute_delta(double actual, double desired)
{
  if (fabs(desired) < 1e-2) {
    return desired - actual;
  }
  return (desired - actual) / desired;
}


void PumpPlugin::
update(void)
{
  bool ok(true);
  
  ros::WallTime const wall_now(ros::WallTime::now());
  bool const debug(wall_now >= debug_publish_time_);
  if (debug) {
    debug_publish_time_ = wall_now + debug_publish_delay_;
  }
  
  //////////////////////////////////////////////////
  // Pump robot state from ROS to WBC, taking care not to swamp the
  // communication channel: only send after receiving, or in the first
  // iteration.
  
  if (ready_to_send_) {
    
    for (size_t ii(0); ii < ndof_; ++ii) {
      mq_robot_api_->pos()[ii] = controlled_joint_[ii]->position_;
      mq_robot_api_->vel()[ii] = controlled_joint_[ii]->velocity_;
      mq_robot_api_->force()[ii] = controlled_joint_[ii]->measured_effort_;
    }
    *(mq_robot_api_->dutyCycle()) = 0;
    for (size_t ii(0); ii < N_DUTY_CYCLE; ++ii) {
      *(mq_robot_api_->dutyCycle()) += duty_cycle_array_[ii];
    }
    *(mq_robot_api_->dutyCycle()) /= N_DUTY_CYCLE;
    
    if (0 > mq_robot_api_->sendState()) {
      ok = false;
      ROS_ERROR ("robot sendState %s", mq_robot_api_->errstr());
    }
    
    if (debug) {
      debug_state_msg_.joint_name.resize(ndof_);
      debug_state_msg_.position.resize(ndof_);
      debug_state_msg_.velocity.resize(ndof_);
      debug_state_msg_.force.resize(ndof_);
      debug_state_msg_.command_delta.resize(ndof_);
      for (size_t ii(0); ii < ndof_; ++ii) {
	debug_state_msg_.joint_name[ii] = controlled_joint_[ii]->joint_->name;
	debug_state_msg_.position[ii] = controlled_joint_[ii]->position_;
	debug_state_msg_.velocity[ii] = controlled_joint_[ii]->velocity_;
	debug_state_msg_.force[ii] = controlled_joint_[ii]->measured_effort_;
	
	// In order to compute the difference between the desired and
	// executed motor command, we find the closest matching one in
	// the command history. This is an optimistic estimate based
	// on the fact that there is some delay from the moment we set
	// the commanded effort and when the measured motor current
	// information makes its way back here as measured effort.
	double const effort(controlled_joint_[ii]->measured_effort_);
	double * cptr(command_history_ + N_COMMAND_HISTORY * ii);
	double delta(compute_delta(effort, *cptr));
	for (size_t jj(1); jj < N_COMMAND_HISTORY; ++jj, ++cptr) {
	  double const dd(compute_delta(effort, *cptr));
	  if (fabs(dd) < fabs(delta)) {
	    delta = dd;
	  }
	}
	debug_state_msg_.command_delta[ii] = delta;
	
      }
      debug_state_pub_.publish(debug_state_msg_);
    }
    
    ready_to_send_ = false;	// always reset to avoid swamping the mqueue
  }

  //////////////////////////////////////////////////
  // Pump torque command from WBC to ROS.
  
  if (ok) {
    
    int const status(mq_robot_api_->receiveCom());
    if (0 > status) {
      ok = false;
      ROS_ERROR ("robot receiveCom %s", mq_robot_api_->errstr());
    }
    else if (0 == status) {
      ready_to_send_ = true;
      duty_cycle_array_[tick_ % N_DUTY_CYCLE] = 1;
    }
    else {
      duty_cycle_array_[tick_ % N_DUTY_CYCLE] = 0;
    }
    
    if (debug && (0 == status)) {
      debug_command_msg_.joint_name.resize(ndof_);
      debug_command_msg_.command.resize(ndof_);
      for (size_t ii(0); ii < ndof_; ++ii) {
	debug_command_msg_.joint_name[ii] = controlled_joint_[ii]->joint_->name;
	debug_command_msg_.command[ii] = mq_robot_api_->comLatch()[ii];
      }
      debug_command_pub_.publish(debug_command_msg_);
    }
    
  }
  
  //////////////////////////////////////////////////
  // Write the command to the motors.
  
  if (com_enabled_ && ok) {
    size_t const offset(tick_ % N_COMMAND_HISTORY);
    for (size_t ii(0); ii < ndof_; ++ii) {
      // comLatch is the last received com, either from this tick or a
      // previous one.
      controlled_joint_[ii]->commanded_effort_ = mq_robot_api_->comLatch()[ii];
      command_history_[ii * N_COMMAND_HISTORY + offset] = mq_robot_api_->comLatch()[ii];
    }
  }
  else {
    size_t const offset(tick_ % N_COMMAND_HISTORY);
    for (size_t ii(0); ii < ndof_; ++ii) {
      controlled_joint_[ii]->commanded_effort_ = 0; // on PR2 it should be safe to send zero torques on error
      command_history_[ii * N_COMMAND_HISTORY + offset] = 0;
    }
  }
  
  ++tick_;
}


static bool str_to_bool(char const * excprefix, string const & str, bool emptyval) throw(std::runtime_error)
{
  if ("n" == str) {
    return false;
  }
  if ("y" == str) {
    return true;
  }
  if ("" == str) {
    return emptyval;
  }
  throw std::runtime_error(excprefix + str + " (please say 'y' or 'n', or leave it empty for the default)");
}


bool PumpPlugin::
init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle & nn)
{
  try {
    string com_enabled_val;
    if ( ! nn.getParam("/wbc_pr2_ctrl_pump/com_enabled", com_enabled_val)) {
      com_enabled_ = false;
    }
    com_enabled_ = str_to_bool("PumpPlugin::init(): /wbc_pr2_ctrl_pump/com_enabled ", com_enabled_val, false);
    ROS_INFO ("com_enabled set to %s", com_enabled_ ? "TRUE" : "FALSE");
    
    string mq_blocking_val;
    if ( ! nn.getParam("/wbc_pr2_ctrl_pump/mq_blocking", mq_blocking_val)) {
      mq_blocking_ = false;
    }
    mq_blocking_ = str_to_bool("PumpPlugin::init(): /wbc_pr2_ctrl_pump/mq_blocking ", mq_blocking_val, false);
    ROS_INFO ("mq_blocking set to %s", mq_blocking_ ? "TRUE" : "FALSE");
    
    static size_t const n_tao_roots(1);
    model_.initFromURDF(nn, robot->model_->robot_model_, n_tao_roots);
    
    controlled_joint_.clear();	// paranoid
    for (size_t ii(0); ii < model_.tao_trees_[0]->info.size(); ++ii) {
      pr2_mechanism_model::JointState * joint(robot->getJointState(model_.tao_trees_[0]->info[ii].joint_name));
      if ( ! joint) { // "never" happens because this is where the joint names come from in the first place...
	throw runtime_error("weird, no joint called `" + model_.tao_trees_[0]->info[ii].joint_name
			    + "' in the pr2_mechanism_model???");
      }
      controlled_joint_.push_back(joint);
    }
    
    static bool const unlink_mqueue(true);
    ndof_ = controlled_joint_.size();
    mq_robot_api_ = new wbc_pr2_ctrl::MQRobotAPI(unlink_mqueue);
    mq_robot_api_->init(mq_blocking_, "wbc_pr2_ctrl_r2s", "wbc_pr2_ctrl_s2r", ndof_, ndof_, ndof_, ndof_);
    ready_to_send_ = true;
    
    command_history_ = new double[N_COMMAND_HISTORY * ndof_];
    memset(command_history_, 0, sizeof(double) * N_COMMAND_HISTORY * ndof_);
    
    ROS_INFO ("successfully initialized the \"pump\"");
    
  }
  
  catch (std::exception const & ee) {
    ROS_ERROR ("PumpPlugin::init(): EXCEPTION: %s", ee.what());
    return false;
  }
  
  debug_state_pub_ = nn.advertise<wbc_pr2_ctrl::PumpDebugState>("debug_state", 100);
  debug_command_pub_ = nn.advertise<wbc_pr2_ctrl::PumpDebugCommand>("debug_command", 100);
  debug_publish_time_ = ros::WallTime::now() + debug_publish_delay_;
  
  return true;
}


PLUGINLIB_REGISTER_CLASS (PumpPlugin, PumpPlugin, pr2_controller_interface::Controller)
