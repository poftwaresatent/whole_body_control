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
   \file opspace_planar_controller.h An very simple opspace controller
   for testing. Given two joints of the robot, it treats them like a
   two-link planar arm of given segment lengths, and does a 2D
   J-transpose-F with velocity stauration, while providing joint-space
   position control for all the others.
   
   \author Roland Philippsen
*/

#ifndef WBC_PR2_CTRL_OPSPACE_PLANAR_CONTROLLER_H
#define WBC_PR2_CTRL_OPSPACE_PLANAR_CONTROLLER_H

#include <jspace/Model.hpp>
#include <jspace/controller_library.hpp>

namespace wbc_pr2_ctrl {

  class OpspacePlanarController
    : public jspace::Controller
  {
  public:
    OpspacePlanarController(std::string const & q1_name,
			    bool q1_inverted,
			    double l1_length,
			    std::string const & q2_name,
			    bool q2_inverted,
			    double l2_length,
			    double op_kp, double op_kd, double op_vmax,
			    jspace::Vector const & default_kp,
			    jspace::Vector const & default_kd);
    
    virtual ~OpspacePlanarController();

    /** \todo This is the only part that depends on ROS, and thus it
	would be better to put it in a separate file in order to keep
	OpspacePlanarController completely middleware independent. */
    static OpspacePlanarController * create(std::string const & param_root,
					    jspace::Vector const & default_kp,
					    jspace::Vector const & default_kd);
    
    virtual jspace::controller_info_getter_s const * getInfo() const;
    virtual jspace::Status init(jspace::Model const & model);
    
    virtual jspace::Status setGoal(jspace::Vector const & goal);
    virtual jspace::Status getGoal(jspace::Vector & goal) const;
    virtual jspace::Status getActual(jspace::Vector & actual) const;
    
    virtual jspace::Status setGains(jspace::Vector const & kp, jspace::Vector const & kd);
    virtual jspace::Status getGains(jspace::Vector & kp, jspace::Vector & kd) const;
    
    virtual jspace::Status latch(jspace::Model const & model);
    virtual jspace::Status computeCommand(jspace::Model const & model, jspace::Vector & tau);
    
    /** \todo Put something similar to this, but optional, into the base class. */
    bool getDebug(jspace::Vector & velocity,
		  std::vector<std::string> & ext_name,
		  jspace::Vector & ext) const;
    
    
  protected:
    std::string const q1_name_;
    bool const q1_inverted_;
    double const l1_length_;
    std::string const q2_name_;
    bool const q2_inverted_;
    double const l2_length_;
    
    double const gdist_sqr_min_;
    double const gdist_sqr_max_;
    
    double op_kp_;
    double op_kd_;
    double op_kp_by_kd_;
    double op_vmax_;
    jspace::JointGoalController jgoal_controller_;
    
    mutable jspace::controller_info_getter_s const * info_;
    
    ssize_t q1_index_;
    ssize_t q2_index_;
    size_t ndof_;
    jspace::Vector op_goal_;
    jspace::Vector op_position_;
    jspace::Matrix jacobian_;
    
    // Could be kept purely local to computeCommand(), but it's nice
    // to have them available for debugging.
    jspace::Vector op_velocity_;
    jspace::Vector op_fstar_;
  };
  
}

#endif // WBC_PR2_CTRL_OPSPACE_PLANAR_CONTROLLER_H
