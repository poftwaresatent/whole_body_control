/*
 * PR2 controller plugin for Stanford-WBC http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file wbc_plugin.cpp
   \brief Plugin that directly implements a whole-body controller.
   \author Roland Philippsen
*/

#include <wbc_pr2_ctrl/TaskPostureUI.h>
#include <pr2_controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <jspace/tao_util.hpp>
#include <jspace/Model.hpp>
#include <wbc_urdf/Model.hpp>
#include <ros/console.h>
#include <tao/dynamics/taoDNode.h>
#include <Eigen/SVD>

using namespace std;


static bool stepTaskPosture(jspace::Model const & model,
			    taoDNode const * end_effector,
			    jspace::Vector const & local_control_point,
			    jspace::Vector const & task_goal,
			    jspace::Vector const & task_kp,
			    jspace::Vector const & task_kd,
			    jspace::Vector const & posture_goal,
			    jspace::Vector const & posture_kp,
			    jspace::Vector const & posture_kd,
			    jspace::Vector & tau);


struct ctrl_to_ui_s {
  jspace::State state;
  jspace::Vector tau;
};


struct ui_to_ctrl_s {
  ui_to_ctrl_s & operator = (ui_to_ctrl_s const & rhs) {
    if (&rhs != this) {
      local_control_point = rhs.local_control_point;
      task_goal = rhs.task_goal;
      task_kp = rhs.task_kp;
      task_kd = rhs.task_kd;
      posture_goal = rhs.posture_goal;
      posture_kp = rhs.posture_kp;
      posture_kd = rhs.posture_kd;
    }
    return *this;
  }
  
  jspace::Vector local_control_point;
  jspace::Vector task_goal;
  jspace::Vector task_kp;
  jspace::Vector task_kd;
  jspace::Vector posture_goal;
  jspace::Vector posture_kp;
  jspace::Vector posture_kd;
};


static size_t const NBUF(2);


class WBCPlugin
  : public pr2_controller_interface::Controller
{
public:
  WBCPlugin();
  virtual ~WBCPlugin();
  
  virtual void update(void);
  virtual bool init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle &nn);
  
  bool uiCallback(wbc_pr2_ctrl::TaskPostureUI::Request & request,
		  wbc_pr2_ctrl::TaskPostureUI::Response & response);
  
  std::vector<pr2_mechanism_model::JointState *> controlled_joint_;
  jspace::ros::Model ros_model_;
  size_t ndof_;
  jspace::Model model_;
  taoDNode const * end_effector_;
  int tick_;
  
  size_t ctrl_to_ui_tick_;
  ctrl_to_ui_s ctrl_to_ui_data_[NBUF];
  
  size_t ui_to_ctrl_tick_;
  ui_to_ctrl_s ui_to_ctrl_data_[NBUF];
  
  ros::ServiceServer ui_server_;
};


WBCPlugin::
WBCPlugin()
  : ros_model_("/wbc_pr2_ctrl/"),
    ndof_(0),
    tick_(0),
    ctrl_to_ui_tick_(1),	// start at one because we need tick-1
    ui_to_ctrl_tick_(1)		// start at one because we need tick-1
{
}


WBCPlugin::
~WBCPlugin()
{
}


static inline size_t clean(size_t tick)
{
  return tick % NBUF;
}


static inline size_t dirty(size_t tick)
{
  return (tick - 1) % NBUF;
}


void WBCPlugin::
update(void)
{
  ctrl_to_ui_s & out(ctrl_to_ui_data_[dirty(ctrl_to_ui_tick_)]);
  ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
  
  //////////////////////////////////////////////////
  // update state
  
  ros::WallTime const wall_now(ros::WallTime::now());
  out.state.time_sec_ = wall_now.sec;
  out.state.time_usec_ = wall_now.nsec / 1000;
  
  for (size_t ii(0); ii < ndof_; ++ii) {
    out.state.position_[ii] = controlled_joint_[ii]->position_;
    out.state.velocity_[ii] = controlled_joint_[ii]->velocity_;
    out.state.force_[ii] = controlled_joint_[ii]->measured_effort_;
  }
  model_.update(out.state);
  
  //////////////////////////////////////////////////
  // compute control torques
  
  bool const ok(stepTaskPosture(model_,
				end_effector_,
				in.local_control_point,
				in.task_goal,
				in.task_kp,
				in.task_kd,
				in.posture_goal,
				in.posture_kp,
				in.posture_kd,
				out.tau));
  
  //////////////////////////////////////////////////
  // send torques to motors
  
  if (ok) {
    for (size_t ii(0); ii < ndof_; ++ii) {
      controlled_joint_[ii]->commanded_effort_ = out.tau[ii];
    }
  }
  else {
    for (size_t ii(0); ii < ndof_; ++ii) {
      out.tau[ii] = 0;
      controlled_joint_[ii]->commanded_effort_ = 0;
    }
  }
  
  // I guess ctrl_to_ui_tick is synonymous with just tick... also note
  // that ui_to_ctrl_tick gets updated in uiCallback()
  ++ctrl_to_ui_tick_;
  ++tick_;
}


bool WBCPlugin::
init(pr2_mechanism_model::RobotState * robot, ros::NodeHandle & nn)
{
  try {
    
    ROS_INFO ("creating TAO tree from URDF");
    static size_t const n_tao_roots(1);
    ros_model_.initFromURDF(nn, robot->model_->robot_model_, n_tao_roots);
    
    ROS_INFO ("retrieving controlled joints");
    controlled_joint_.clear();	// paranoid
    for (size_t ii(0); ii < ros_model_.tao_trees_[0]->info.size(); ++ii) {
      pr2_mechanism_model::JointState *
	joint(robot->getJointState(ros_model_.tao_trees_[0]->info[ii].joint_name));
      if ( ! joint) {
	// "never" happens because this is where the joint names come from in the first place...
	throw runtime_error("weird, no joint called `"
			    + ros_model_.tao_trees_[0]->info[ii].joint_name
			    + "' in the pr2_mechanism_model???");
      }
      controlled_joint_.push_back(joint);
    }
    ndof_ = controlled_joint_.size();
    
    ROS_INFO ("creating jspace model from TAO");
    {
      std::ostringstream msg;
      if (0 != model_.init(ros_model_.tao_trees_[0], 0, &msg)) {
	throw std::runtime_error("jspace::Model::init() failed: " + msg.str());
      }
    }
    if (model_.getNDOF() != ndof_) {
      std::ostringstream msg;
      msg << "weird, jspace::Model::getNDOF() says "
	  << model_.getNDOF() << " but we have " << ndof_ << "DOF";
      throw std::runtime_error(msg.str());
    }
    
    end_effector_ = model_.getNodeByName("l_wrist_roll_link");
    if ( ! end_effector_) {
      throw std::runtime_error("no l_wrist_roll_link in model (MAKE THIS RUNTIME CONFIGURABLE)");
    }
    
    for (size_t ii(0); ii < NBUF; ++ii) {
      ui_to_ctrl_data_[ii].local_control_point = jspace::Vector::Zero(3);
      ui_to_ctrl_data_[ii].local_control_point << 0.0 , 0.1 , 0.0;
      ui_to_ctrl_data_[ii].task_goal =     0.2 * jspace::Vector::Ones(3);
      ui_to_ctrl_data_[ii].task_kp =     100.0 * jspace::Vector::Ones(3);
      ui_to_ctrl_data_[ii].task_kd =      20.0 * jspace::Vector::Ones(3);
      ui_to_ctrl_data_[ii].posture_goal = 20.0 * M_PI / 180.0 * jspace::Vector::Ones(ndof_);
      ui_to_ctrl_data_[ii].posture_kp =  100.0 * jspace::Vector::Ones(ndof_);
      ui_to_ctrl_data_[ii].posture_kd =   20.0 * jspace::Vector::Ones(ndof_);
    }
    
    ROS_INFO ("marking gravity-compensated joints");
    std::vector<std::string>::const_iterator
      gclink(ros_model_.gravity_compensated_links_.begin());
    for (/**/; gclink != ros_model_.gravity_compensated_links_.end(); ++gclink) {
      taoDNode const * node(model_.getNodeByName(*gclink));
      if ( ! node) {
	throw std::runtime_error("gravity-compensated link " + *gclink
				 + " is not part of the jspace::Model");
      }
      int const id(node->getID());
      model_.disableGravityCompensation(id, true);
      ROS_INFO ("disabled gravity compensation for link %s (ID %d)", gclink->c_str(), id);
    }
    
    for (size_t ii(0); ii < NBUF; ++ii) {
      ctrl_to_ui_data_[ii].state.init(ndof_, ndof_, ndof_);
      ctrl_to_ui_data_[ii].tau = jspace::Vector::Zero(ndof_);
    }
    tick_ = 0;
    
    ROS_INFO ("wbc_plugin ready to rock!");
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("WBCPlugin::init(): EXCEPTION: %s", ee.what());
    return false;
  }
  
  ui_server_ = nn.advertiseService("ui", &WBCPlugin::uiCallback, this);
  
  return true;
}


bool WBCPlugin::
uiCallback(wbc_pr2_ctrl::TaskPostureUI::Request & request,
	   wbc_pr2_ctrl::TaskPostureUI::Response & response)
{
  response.ok = true;
  
  switch (request.mode) {
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::SET_TASK_GOAL:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      ui_to_ctrl_s & out(ui_to_ctrl_data_[dirty(ui_to_ctrl_tick_)]);
      out = in;
      if ( ! request.value.empty()) {
	if (3 == request.value.size()) {
	  jspace::convert(request.value, out.task_goal);
	}
	else {
	  response.ok = false;
	  response.errstr = "invalid task goal dimension";
	}
      }
      ++ui_to_ctrl_tick_;
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::SET_TASK_KP:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      ui_to_ctrl_s & out(ui_to_ctrl_data_[dirty(ui_to_ctrl_tick_)]);
      out = in;
      if ( ! request.value.empty()) {
	if (3 == request.value.size()) {
	  jspace::convert(request.value, out.task_kp);
	}
	else {
	  response.ok = false;
	  response.errstr = "invalid task kp dimension";
	}
      }
      ++ui_to_ctrl_tick_;
      break;
    }

  case wbc_pr2_ctrl::TaskPostureUI::Request::SET_TASK_KD:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      ui_to_ctrl_s & out(ui_to_ctrl_data_[dirty(ui_to_ctrl_tick_)]);
      out = in;
      if ( ! request.value.empty()) {
	if (3 == request.value.size()) {
	  jspace::convert(request.value, out.task_kd);
	}
	else {
	  response.ok = false;
	  response.errstr = "invalid task kd dimension";
	}
      }
      ++ui_to_ctrl_tick_;
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::SET_POSTURE_GOAL:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      ui_to_ctrl_s & out(ui_to_ctrl_data_[dirty(ui_to_ctrl_tick_)]);
      out = in;
      if ( ! request.value.empty()) {
	if (ndof_ == request.value.size()) {
	  jspace::convert(request.value, out.posture_goal);
	}
	else {
	  response.ok = false;
	  response.errstr = "invalid posture goal dimension";
	}
      }
      ++ui_to_ctrl_tick_;
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::SET_POSTURE_KP:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      ui_to_ctrl_s & out(ui_to_ctrl_data_[dirty(ui_to_ctrl_tick_)]);
      out = in;
      if ( ! request.value.empty()) {
	if (ndof_ == request.value.size()) {
	  jspace::convert(request.value, out.posture_kp);
	}
	else {
	  response.ok = false;
	  response.errstr = "invalid posture kp dimension";
	}
      }
      ++ui_to_ctrl_tick_;
      break;
    }

  case wbc_pr2_ctrl::TaskPostureUI::Request::SET_POSTURE_KD:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      ui_to_ctrl_s & out(ui_to_ctrl_data_[dirty(ui_to_ctrl_tick_)]);
      out = in;
      if ( ! request.value.empty()) {
	if (ndof_ == request.value.size()) {
	  jspace::convert(request.value, out.posture_kd);
	}
	else {
	  response.ok = false;
	  response.errstr = "invalid posture kd dimension";
	}
      }
      ++ui_to_ctrl_tick_;
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::GET_TASK_GOAL:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      jspace::convert(in.task_goal, response.value);
      for (size_t ii(0); ii < 3; ++ii) {
	response.lower_bound.push_back(-2); // should not hardcode this... ah well.
	response.upper_bound.push_back(2);
	response.unit.push_back("m");
      }
      response.name.push_back("EE pos x");
      response.name.push_back("EE pos y");
      response.name.push_back("EE pos z");
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::GET_TASK_KP:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      jspace::convert(in.task_kp, response.value);
      for (size_t ii(0); ii < 3; ++ii) {
	response.lower_bound.push_back(0);
	response.upper_bound.push_back(1000); // should not hardcode this... ah well.
      }
      response.name.push_back("kp x");
      response.name.push_back("kp y");
      response.name.push_back("kp z");
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::GET_TASK_KD:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      jspace::convert(in.task_kd, response.value);
      for (size_t ii(0); ii < 3; ++ii) {
	response.lower_bound.push_back(0);
	response.upper_bound.push_back(63); // should not hardcode this... ah well.
      }
      response.name.push_back("kd x");
      response.name.push_back("kd y");
      response.name.push_back("kd z");
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::GET_POSTURE_GOAL:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      jspace::convert(in.posture_goal, response.value);
      for (size_t ii(0); ii < ndof_; ++ii) {
	response.lower_bound.push_back(-2 * M_PI); // should not hardcode this... ah well.
	response.upper_bound.push_back(2 * M_PI);
	response.unit.push_back("rad");
	ostringstream msg;
	msg << "joint pos " << ii;
	response.name.push_back(msg.str());
      }
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::GET_POSTURE_KP:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      jspace::convert(in.posture_kp, response.value);
      for (size_t ii(0); ii < ndof_; ++ii) {
	response.lower_bound.push_back(0);
	response.upper_bound.push_back(1000); // should not hardcode this... ah well.
	ostringstream msg;
	msg << "joint kp " << ii;
	response.name.push_back(msg.str());
      }
      break;
    }
    
  case wbc_pr2_ctrl::TaskPostureUI::Request::GET_POSTURE_KD:
    {
      ui_to_ctrl_s const & in(ui_to_ctrl_data_[clean(ui_to_ctrl_tick_)]);
      jspace::convert(in.posture_kd, response.value);
      for (size_t ii(0); ii < ndof_; ++ii) {
	response.lower_bound.push_back(0);
	response.upper_bound.push_back(63); // should not hardcode this... ah well.
	ostringstream msg;
	msg << "joint kd " << ii;
	response.name.push_back(msg.str());
      }
      break;
    }
    
  default:
    {
      ostringstream msg;
      msg << "invalid mode: " << request.mode;
      response.ok = false;
      response.errstr = msg.str();
      break;
    }
  }
  
  return true;
}


static void pseudoInverse(jspace::Matrix const & matrix,
			  double sigmaThreshold,
			  jspace::Matrix & invMatrix)
{
  Eigen::SVD<jspace::Matrix> svd(matrix);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  jspace::Matrix invS;
  invS = jspace::Matrix::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    }
  }
  invMatrix = svd.matrixU() * invS * svd.matrixU().transpose();
}


bool stepTaskPosture(jspace::Model const & model,
		     taoDNode const * end_effector,
		     jspace::Vector const & local_control_point,
		     jspace::Vector const & task_goal,
		     jspace::Vector const & task_kp,
		     jspace::Vector const & task_kd,
		     jspace::Vector const & posture_goal,
		     jspace::Vector const & posture_kp,
		     jspace::Vector const & posture_kd,
		     jspace::Vector & tau)
{
  //////////////////////////////////////////////////
  // sanity checks
  
  if (local_control_point.rows() != 3) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid local_control_point dimension %d",
	       local_control_point.rows());
    return false;
  }
  if (task_goal.rows() != 3) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid task_goal dimension %d",
	       task_goal.rows());
    return false;
  }
  if (task_kp.rows() != 3) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid task_kp dimension %d",
	       task_kp.rows());
    return false;
  }
  if (task_kd.rows() != 3) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid task_kd dimension %d",
	       task_kd.rows());
    return false;
  }
  
  size_t const ndof(model.getNDOF());
  if (posture_goal.rows() != ndof) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid posture_goal dimension %d",
	       posture_goal.rows());
    return false;
  }
  if (posture_kp.rows() != ndof) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid posture_kp dimension %d",
	       posture_kp.rows());
    return false;
  }
  if (posture_kd.rows() != ndof) {
    ROS_ERROR ("WBCPlugin::stepTaskPosture(): invalid posture_kd dimension %d",
	       posture_kd.rows());
    return false;
  }

  //////////////////////////////////////////////////
  // task
  
  jspace::Transform eepos;
  model.computeGlobalFrame(end_effector,
			   local_control_point[0],
			   local_control_point[1],
			   local_control_point[2],
			   eepos);
  
  jspace::Matrix Jfull;
  model.computeJacobian(end_effector,
			eepos.translation()[0],
			eepos.translation()[1],
			eepos.translation()[2],
			Jfull);
  jspace::Matrix Jx(Jfull.block(0, 0, 3, ndof));
  jspace::Matrix invA;
  model.getInverseMassInertia(invA);
  jspace::Matrix invLambda(Jx * invA * Jx.transpose());
  jspace::Matrix Lambda;
  pseudoInverse(invLambda, 1e-3, Lambda);
  
  jspace::Vector poserror(eepos.translation() - task_goal);
  jspace::Vector velerror(Jx * model.getState().velocity_); // desired velocity == zero

  cerr << "==================================================\n";
  jspace::pretty_print(Jx, cerr, "Jx", "  ");
  jspace::pretty_print(Lambda, cerr, "Lambda", "  ");
  jspace::pretty_print(poserror, cerr, "poserror", "  ");
  jspace::pretty_print(velerror, cerr, "velerror", "  ");
  jspace::pretty_print(task_kp, cerr, "task_kp", "  ");
  jspace::pretty_print(task_kd, cerr, "task_kd", "  ");
  
  jspace::Vector tau_task(Jx.transpose() * (-Lambda)
			  * (   task_kp.cwise() * poserror
			      + task_kd.cwise() * velerror));
  
  cerr << "--------------------------------------------------\n";
  jspace::pretty_print(tau_task, cerr, "tau_task", "  ");
  
  //////////////////////////////////////////////////
  // posture
  
  jspace::Matrix Jbar(invA * Jx.transpose() * Lambda);
  jspace::Matrix nullspace(jspace::Matrix::Identity(ndof, ndof) - Jbar * Jx);
  jspace::Matrix invLambda_p(nullspace * invA);
  jspace::Matrix Lambda_p;
  pseudoInverse(invLambda_p, 1e-3, Lambda_p);
  
  cerr << "--------------------------------------------------\n";
  jspace::pretty_print(nullspace, cerr, "nullspace", "  ");
  jspace::pretty_print(Lambda_p, cerr, "Lambda_p", "  ");
  jspace::pretty_print(posture_goal, cerr, "posture_goal", "  ");
  jspace::pretty_print(model.getState().position_, cerr, "position", "  ");
  jspace::pretty_print(posture_kp, cerr, "posture_kp", "  ");
  jspace::pretty_print(posture_kd, cerr, "posture_kd", "  ");

  jspace::Vector tau_posture(nullspace.transpose() * (-Lambda_p)
			     * (  posture_kp.cwise() * (model.getState().position_ - posture_goal)
				+ posture_kd.cwise() *  model.getState().velocity_));
  
  cerr << "--------------------------------------------------\n";
  jspace::pretty_print(tau_posture, cerr, "tau_posture", "  ");
  
  //////////////////////////////////////////////////
  // sum it up...
  
  jspace::Vector gg;
  model.getGravity(gg);
  tau = tau_task + tau_posture + gg;
  
  cerr << "--------------------------------------------------\n";
  jspace::pretty_print(tau, cerr, "tau", "  ");
}


PLUGINLIB_REGISTER_CLASS (WBCPlugin, WBCPlugin, pr2_controller_interface::Controller)
