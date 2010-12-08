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
   \file opspace_planar_controller.cpp   
   \author Roland Philippsen
*/

#include <wbc_pr2_ctrl/opspace_planar_controller.h>

#include <ros/ros.h>
#include <jspace/Model.hpp>
#include <tao/dynamics/taoNode.h>


namespace {
  
  struct OpspacePlanar_info_getter_s
    : public jspace::controller_info_getter_s
  {
    ssize_t const q1_index_;
    ssize_t const q2_index_;
    double const max_reach_;
    jspace::controller_info_getter_s const * const jspace_info_;
    
    OpspacePlanar_info_getter_s(ssize_t q1_index, ssize_t q2_index, double max_reach,
				jspace::controller_info_getter_s const * jspace_info)
      : q1_index_(q1_index),
	q2_index_(q2_index),
	max_reach_(max_reach),
	jspace_info_(jspace_info)
    {
    }
    
    virtual void getDOFNames(jspace::Model const & model, std::vector<std::string> & names) const {
      names.clear();
      names.push_back("position_x");
      names.push_back("position_y");
      std::vector<std::string> jnames;
      jspace_info_->getDOFNames(model, jnames);
      for (ssize_t ii(0); ii < static_cast<ssize_t>(jnames.size()); ++ii) {
	if ((ii != q1_index_) && (ii != q2_index_)) {
	  names.push_back(jnames[ii]);
	}
      }
    }
    
    virtual void getDOFUnits(jspace::Model const & model, std::vector<std::string> & names) const {
      names.clear();
      names.push_back("m");
      names.push_back("m");
      std::vector<std::string> jnames;
      jspace_info_->getDOFUnits(model, jnames);
      for (ssize_t ii(0); ii < static_cast<ssize_t>(jnames.size()); ++ii) {
	if ((ii != q1_index_) && (ii != q2_index_)) {
	  names.push_back(jnames[ii]);
	}
      }
    }

    virtual void getGainNames(jspace::Model const & model, std::vector<std::string> & names) const {
      names.clear();
      names.push_back("position");
      names.push_back("vmax [mm/s]");
      std::vector<std::string> jnames;
      jspace_info_->getGainNames(model, jnames);
      for (ssize_t ii(0); ii < static_cast<ssize_t>(jnames.size()); ++ii) {
	if ((ii != q1_index_) && (ii != q2_index_)) {
	  names.push_back(jnames[ii]);
	}
      }
    }
    
    virtual void getLimits(jspace::Model const & model,
			   jspace::Vector & limits_lower,
			   jspace::Vector & limits_upper) const
    {
      jspace::Vector jlower;
      jspace::Vector jupper;
      jspace_info_->getLimits(model, jlower, jupper);
      limits_lower.resize(jlower.size());
      limits_lower[0] = -max_reach_;
      limits_lower[1] = -max_reach_;
      limits_upper.resize(jupper.size());
      limits_upper[0] =  max_reach_;
      limits_upper[1] =  max_reach_;
      jspace_info_->getLimits(model, jlower, jupper);
      ssize_t ii(2);
      ssize_t jj(0);
      for (/**/; jj < jlower.size(); ++jj) {
	if ((jj != q1_index_) && (jj != q2_index_)) {
	  limits_lower[ii] = jlower[jj];
	  limits_upper[ii] = jupper[jj];
	  ++ii;
	}
      }
    }
    
  };
  
  
  static bool str_to_bool(std::string const & excprefix,
			  std::string const & str,
			  bool emptyval) throw(std::runtime_error)
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
    throw std::runtime_error(excprefix + " `" + str + "' (please say 'y' or 'n', or leave it empty for the default)");
  }
  
}


namespace wbc_pr2_ctrl {
  
  OpspacePlanarController::
  OpspacePlanarController(std::string const & q1_name,
			  bool q1_inverted,
			  double l1_length,
			  std::string const & q2_name,
			  bool q2_inverted,
			  double l2_length,
			  double op_kp, double op_kd, double op_vmax,
			  jspace::Vector const & default_kp,
			  jspace::Vector const & default_kd)
    : jspace::Controller(),
      q1_name_(q1_name), 
      q1_inverted_(q1_inverted), 
      l1_length_(l1_length),
      q2_name_(q2_name),
      q2_inverted_(q2_inverted),
      l2_length_(l2_length),
      gdist_sqr_min_(pow(l1_length - l2_length, 2)),
      gdist_sqr_max_(pow(l1_length + l2_length, 2)),
      op_kp_(op_kp),
      op_kd_(op_kd),
      op_kp_by_kd_(fabs(op_kd) < 1e-2 ? -1 : op_kp / op_kd),
      op_vmax_(op_vmax),
      jgoal_controller_(jspace::COMP_GRAVITY, default_kp, default_kd),
      info_(0),
      q1_index_(-1),
      q2_index_(-1),
      ndof_(0),
      op_goal_(jspace::Vector::Zero(2)),
      op_position_(jspace::Vector::Zero(2)),
      jacobian_(jspace::Matrix::Identity(2, 2)),
      op_velocity_(jspace::Vector::Zero(2)),
      op_fstar_(jspace::Vector::Zero(2))
  {
  }
  
  
  OpspacePlanarController::
  ~OpspacePlanarController()
  {
    delete info_;
  }
  
  
  jspace::controller_info_getter_s const * OpspacePlanarController::
  getInfo() const
  {
    if ((0 > q1_index_) || (0 > q2_index_)) {
      return 0;			// ERROR: not (properly) initialized
    }
    if ( ! info_) {
      info_ = new OpspacePlanar_info_getter_s(q1_index_, q2_index_,
					      l1_length_ + l2_length_,
					      jgoal_controller_.getInfo());
    }
    return info_;
  }
  
  
  jspace::Status OpspacePlanarController::
  init(jspace::Model const & model)
  {
    jspace::Status status(jgoal_controller_.init(model));
    if ( ! status) {
      return status;
    }
    
    taoDNode * q1_node(model.getNodeByName(q1_name_));
    if ( ! q1_node) {
      status.ok = false;
      status.errstr = "invalid q1_name, no node called `" + q1_name_ + "'";
      return status;
    }
    q1_index_ = q1_node->getID();
    
    taoDNode * q2_node(model.getNodeByName(q2_name_));
    if ( ! q2_node) {
      status.ok = false;
      status.errstr = "invalid q2_name, no node called `" + q2_name_ + "'";
      return status;
    }
    q2_index_ = q2_node->getID();
    
    ndof_ = model.getNDOF();
    
    return status;
  }
  
  
  jspace::Status OpspacePlanarController::
  setGoal(jspace::Vector const & goal)
  {
    if (goal.size() != ndof_) {
      return jspace::Status(false, "NDOF mismatch in provided goal");
    }
    
    double const gdist_sqr(pow(goal[0], 2) + pow(goal[1], 2));
    if (gdist_sqr <= gdist_sqr_min_) {
      return jspace::Status(false, "the goal is too close to the origin");
    }
    if (gdist_sqr >= gdist_sqr_max_) {
      return jspace::Status(false, "the goal is too far from the origin");
    }
    
    op_goal_.coeffRef(0) = goal[0];
    op_goal_.coeffRef(1) = goal[1];
    jspace::Vector jgoal(ndof_);
    ssize_t ii(2);
    ssize_t jj(0);
    for (/**/; jj < ndof_; ++jj) {
      if ((jj == q1_index_) || (jj == q2_index_)) {
	jgoal[jj] = 0;
      }
      else {
	jgoal[jj] = goal[ii];
	++ii;
      }
    }
    
    return jgoal_controller_.setGoal(jgoal);
  }
  
  
  jspace::Status OpspacePlanarController::
  getGoal(jspace::Vector & goal) const
  {
    jspace::Vector jgoal;
    jspace::Status status(jgoal_controller_.getGoal(jgoal));
    if ( ! status) {
      return status;
    }
    
    goal.resize(ndof_);
    goal[0] = op_goal_[0];
    goal[1] = op_goal_[1];
    for (ssize_t ii(0), off(2); ii < static_cast<ssize_t>(ndof_); ++ii) {
      if ((ii == q1_index_) || (ii == q2_index_)) {
	--off;
      }
      else {
	if (ii + off >= static_cast<ssize_t>(ndof_)) {
	  status.ok = false;
	  status.errstr = "bug in OpspacePlanarController::getGoal() vector mixing";
	  return status;
	}
	goal[ii + off] = jgoal[ii];
      }
    }
    
    return status;
  }
  
  
  jspace::Status OpspacePlanarController::
  getActual(jspace::Vector & actual) const
  {
    jspace::Vector jactual;
    jspace::Status status(jgoal_controller_.getActual(jactual));
    if ( ! status) {
      return status;
    }
    
    actual.resize(ndof_);
    actual[0] = op_position_[0];
    actual[1] = op_position_[1];
    for (ssize_t ii(0), off(2); ii < static_cast<ssize_t>(ndof_); ++ii) {
      if ((ii == q1_index_) || (ii == q2_index_)) {
	--off;
      }
      else {
	if (ii + off >= static_cast<ssize_t>(ndof_)) {
	  status.ok = false;
	  status.errstr = "bug in OpspacePlanarController::getActual() vector mixing";
	  return status;
	}
	actual[ii + off] = jactual[ii];
      }
    }
    
    return status;
  }
  
  
  jspace::Status OpspacePlanarController::
  setGains(jspace::Vector const & kp, jspace::Vector const & kd)
  {
    if ((kp.size() != ndof_) || (kd.size() != ndof_)) {
      return jspace::Status(false, "NDOF mismatch in provided gains");
    }
    
    op_kp_ = kp[0];
    op_kd_ = kd[0];
    op_kp_by_kd_ = fabs(op_kd_) < 1e-2 ? -1 : op_kp_ / op_kd_;
    op_vmax_ = kp[0] * 1e-3;
    
    jspace::Vector jkp(ndof_);
    jspace::Vector jkd(ndof_);
    ssize_t ii(2);
    ssize_t jj(0);
    for (/**/; jj < ndof_; ++jj) {
      if ((jj == q1_index_) || (jj == q2_index_)) {
	jkp[jj] = 0;
	jkd[jj] = 0;
      }
      else {
	jkp[jj] = kp[ii];
	jkd[jj] = kd[ii];
	++ii;
      }
    }
    
    return jgoal_controller_.setGains(jkp, jkd);
  }
  
  
  jspace::Status OpspacePlanarController::
  getGains(jspace::Vector & kp, jspace::Vector & kd) const
  {
    jspace::Vector jkp;
    jspace::Vector jkd;
    jspace::Status status(jgoal_controller_.getGains(jkp, jkd));
    if ( ! status) {
      return status;
    }
    
    kp.resize(ndof_);
    kd.resize(ndof_);
    kp[0] = op_kp_;
    kd[1] = op_kd_;
    kp[1] = op_vmax_ * 1e3;
    kd[1] = 0;
    for (ssize_t ii(0), off(2); ii < static_cast<ssize_t>(ndof_); ++ii) {
      if ((ii == q1_index_) || (ii == q2_index_)) {
	--off;
      }
      else {
	if (ii + off >= static_cast<ssize_t>(ndof_)) {
	  status.ok = false;
	  status.errstr = "bug in OpspacePlanarController::getGains() vector mixing";
	  return status;
	}
	kp[ii + off] = jkp[ii];
	kd[ii + off] = jkd[ii];
      }
    }
    
    return status;
  }
  
  
  jspace::Status OpspacePlanarController::
  latch(jspace::Model const & model)
  {
    jspace::Status status(jgoal_controller_.latch(model));
    if ( ! status) {
      return status;
    }
    
    jspace::State const & state(model.getState());
    double q1(state.position_[q1_index_]);
    double q2(state.position_[q2_index_]);
    if (q1_inverted_) {
      q1 = -q1;
    }
    if (q2_inverted_) {
      q2 = -q2;
    }
    
    double const c1(cos(q1));
    double const s1(sin(q1));
    double const c12(cos(q1+q2));
    double const s12(sin(q1+q2));
    op_goal_[0] = l1_length_ * c1 + l2_length_ * c12;
    op_goal_[1] = l1_length_ * s1 + l2_length_ * s12;
    
    return status;
  }
  
  
  jspace::Status OpspacePlanarController::
  computeCommand(jspace::Model const & model, jspace::Vector & tau)
  {
    // First find out what the joint-space controller would do, later
    // overwrite just the two entries that make up our "planar" robot.
    jspace::Status status(jgoal_controller_.computeCommand(model, tau));
    if ( ! status) {
      return status;
    }
    if (tau.size() != ndof_) {
      status.ok = false;
      status.errstr = "whoa, encapsulated jgoal_controller has a different NDOF???";
      return status;
    }
    
    jspace::State const & state(model.getState());
    double q1(state.position_[q1_index_]);
    double q2(state.position_[q2_index_]);
    if (q1_inverted_) {
      q1 = -q1;
    }
    if (q2_inverted_) {
      q2 = -q2;
    }
    
    double const c1(cos(q1));
    double const s1(sin(q1));
    double const c12(cos(q1+q2));
    double const s12(sin(q1+q2));
    op_position_[0] = l1_length_ * c1 + l2_length_ * c12;
    op_position_[1] = l1_length_ * s1 + l2_length_ * s12;
    
    jacobian_.coeffRef(0, 0) = - l1_length_ * s1 - l2_length_ * s12;
    jacobian_.coeffRef(0, 1) =                   - l2_length_ * s12;
    jacobian_.coeffRef(1, 0) =   l1_length_ * c1 + l2_length_ * c12;
    jacobian_.coeffRef(1, 1) =                     l2_length_ * c12;
    
    // op_velocity_ is only really stored as field for debugging...
    jspace::Vector qvel(2); // will probably need some smoothing on qvel
    qvel[0] = state.velocity_[q1_index_];
    qvel[1] = state.velocity_[q2_index_];
    op_velocity_ = jacobian_ * qvel;
    
    // op_fstar_ is only really stored as field for debugging...
    if (0 > op_kp_by_kd_) {
      // fall back on pure proportional control
      op_fstar_ = - op_kp_ * (op_position_ - op_goal_);
    }
    else {
      jspace::Vector vdes(2);
      if (op_vmax_ < 1e-3) {
	vdes = jspace::Vector::Zero(2);
      }
      else {
	vdes = op_kp_by_kd_ * (op_goal_ - op_position_);
	double const vdes_norm(vdes.norm());
	if (vdes_norm > op_vmax_) {
	  vdes *= op_vmax_ / vdes_norm;
	}
      }
      op_fstar_ = - op_kd_ * (op_velocity_ - vdes);
    }
    
    // taustar is the 2DOF torque that we want to send to the planar
    // robot, so we use its values to overwrite what the joint-space
    // controller would have wanted to do with those two.
    jspace::Vector taustar(jacobian_.transpose() * op_fstar_);
    if (q1_inverted_) {
      tau[q1_index_] = -taustar[0];
    }
    else {
      tau[q1_index_] =  taustar[0];
    }
    if (q2_inverted_) {
      tau[q2_index_] = -taustar[1];
    }
    else {
      tau[q2_index_] =  taustar[1];
    }
    
    return status;
  }
  
  
  OpspacePlanarController * OpspacePlanarController::
  create(std::string const & param_root,
	 jspace::Vector const & default_kp,
	 jspace::Vector const & default_kd)
  {
    ros::NodeHandle nn("~");
    OpspacePlanarController * controller(0);
    
    try {
      std::string q1_name;
      if ( ! nn.getParam(param_root + "/q1_name", q1_name)) {
	throw std::runtime_error("missing q1_name parameter");
      }
      
      std::string tmp;
      bool q1_inverted(false);
      if (nn.getParam(param_root + "/q1_inverted", tmp)) {
	q1_inverted = str_to_bool(param_root + "/q1_inverted", tmp, false);
      }
      
      double l1_length;
      if ( ! nn.getParam(param_root + "/l1_length", l1_length)) {
	throw std::runtime_error("missing l1_length parameter");
      }

      std::string q2_name;
      if ( ! nn.getParam(param_root + "/q2_name", q2_name)) {
	throw std::runtime_error("missing q2_name parameter");
      }
      
      bool q2_inverted(false);
      if (nn.getParam(param_root + "/q2_inverted", tmp)) {
	q2_inverted = str_to_bool(param_root + "/q2_inverted", tmp, false);
      }
      
      double l2_length;
      if ( ! nn.getParam(param_root + "/l2_length", l2_length)) {
	throw std::runtime_error("missing l2_length parameter");
      }
      
      double op_kp;
      if ( ! nn.getParam(param_root + "/op_kp", op_kp)) {
	throw std::runtime_error("missing op_kp parameter");
      }
      
      double op_kd;
      if ( ! nn.getParam(param_root + "/op_kd", op_kd)) {
	throw std::runtime_error("missing op_kd parameter");
      }
      
      double op_vmax;
      if ( ! nn.getParam(param_root + "/op_vmax", op_vmax)) {
	throw std::runtime_error("missing op_vmax parameter");
      }
      
      controller = new OpspacePlanarController(q1_name, q1_inverted, l1_length,
					       q2_name, q2_inverted, l2_length,
					       op_kp, op_kd, op_vmax,
					       default_kp, default_kd);
    }
    
    catch (std::exception const & ee) {
      ROS_ERROR ("OpspacePlanarController::create(`%s'): EXCEPTION: %s", param_root.c_str(), ee.what());
      delete controller;
      controller = 0;
    }
    
    return controller;
  }
  
  
  bool OpspacePlanarController::
  getDebug(jspace::Vector & velocity,
	   std::vector<std::string> & ext_name,
	   jspace::Vector & ext)
    const
  {
    velocity = op_velocity_;
    ext_name.resize(2);
    ext_name[0] = "F*_x";
    ext_name[1] = "F*_y";
    ext = op_fstar_;
  }
  
}
