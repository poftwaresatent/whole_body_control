/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen and Luis Sentis
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <opspace/controller_library.hpp>
#include <reflexxes_otg/TypeIOTG.h>
#include <jspace/Model.hpp>
#include <tao/dynamics/taoDNode.h>
#include <tao/dynamics/taoJoint.h>

namespace opspace {
  
  
  /**
     \todo Move this somewhere more visible, e.g. into reflexxes_otg
     or some util directory underneath the utaustin-wbc project or so.
  */
  class TypeIOTGCursor
  {
  public:
    TypeIOTGCursor(size_t ndof, double dt_seconds)
      : otg_(ndof, dt_seconds)
    {
      pos_clean_ = Vector::Zero(ndof);
      vel_clean_ = Vector::Zero(ndof);
      pos_dirty_ = Vector::Zero(ndof);
      vel_dirty_ = Vector::Zero(ndof);
      selection_.resize(ndof);
      for (size_t ii(0); ii < ndof; ++ii) {
	selection_[ii] = true;
      }
    }
    
    int next(Vector const & maxvel,
	     Vector const & maxacc,
	     Vector const & goal)
    {
      int const result(otg_.GetNextMotionState_Position(pos_clean_.data(),
							vel_clean_.data(),
							maxvel.data(),
							maxacc.data(),
							goal.data(),
							selection_.data(),
							pos_dirty_.data(),
							vel_dirty_.data()));
      if (0 <= result) {
	pos_clean_ = pos_dirty_;
	vel_clean_ = vel_dirty_;
      }
      return result;
    }
    
    inline Vector & position()             { return pos_clean_; }
    inline Vector const & position() const { return pos_clean_; }
    inline Vector & velocity()             { return vel_clean_; }
    inline Vector const & velocity() const { return vel_clean_; }
    
  protected:
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> boolvec_t;
    
    TypeIOTG otg_;
    boolvec_t selection_;
    Vector pos_clean_;
    Vector vel_clean_;
    Vector pos_dirty_;
    Vector vel_dirty_;
  };
  
  
  struct task_posture_info_getter_s
    : public jspace::controller_info_getter_s
  {
    virtual void getDOFNames(Model const & model, std::vector<std::string> & names) const {
      int const njoints(model.getNJoints());
      names.resize(njoints + 3);
      names[0] = "position_x";
      names[1] = "position_y";
      names[2] = "position_z";
      for (int ii(0); ii < njoints; ++ii) {
	names[ii + 3] = model.getJointName(ii);
      }
    }
    
    virtual void getDOFUnits(Model const & model, std::vector<std::string> & names) const {
      int const njoints(model.getNJoints());
      names.resize(njoints + 3);
      names[0] = "m";
      names[1] = "m";
      names[2] = "m";
      for (int ii(0); ii < njoints; ++ii) {
	taoDNode const * node(model.getNode(ii));
	taoJoint const * joint(node->getJointList());
	if (0 != dynamic_cast<taoJointRevolute const *>(joint)) {
	  names[ii + 3] = "rad";
	}
	else if (0 != dynamic_cast<taoJointPrismatic const *>(joint)) {
	  names[ii + 3] = "m";
	}
	else {
	  names[ii + 3] = "void";
	}
      }
    }
    
    virtual void getGainNames(Model const & model, std::vector<std::string> & names) const {
      getDOFNames(model, names);
    }
    
    // Grr, yet another place that can really profit from a more
    // generic task parameter approach: why hardcode limits?
    virtual void getLimits(Model const & model, Vector & limits_lower, Vector & limits_upper) const
    {
      Vector jl_lower, jl_upper;
      model.getJointLimits(jl_lower, jl_upper);
      limits_lower = Vector::Zero(jl_lower.rows() + 3);
      limits_lower[0] = -2.0;	// arbitrary...
      limits_lower[1] = -2.0;	// arbitrary...
      limits_lower[2] = -2.0;	// arbitrary...
      limits_lower.block(3, 0, jl_lower.rows(), 1) = jl_lower;
      limits_upper = Vector::Zero(jl_upper.rows() + 3);
      limits_upper[0] = 2.0;	// arbitrary...
      limits_upper[1] = 2.0;	// arbitrary...
      limits_upper[2] = 2.0;	// arbitrary...
      limits_upper.block(3, 0, jl_upper.rows(), 1) = jl_upper;
    }
  };
  
  
  TaskPostureController::
  TaskPostureController()
    : info_getter_(0),
      dt_seconds_(-1.0),
      end_effector_(0)
  {
  }
  
  
  TaskPostureController::
  ~TaskPostureController()
  {
    delete info_getter_;
  }
  
  
  jspace::controller_info_getter_s const * TaskPostureController::
  getInfo() const
  {
    if ( ! info_getter_) {
      info_getter_ = new task_posture_info_getter_s();
    }
    return info_getter_;
  }
  
  
  Status TaskPostureController::
  init(Model const & model)
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  setGoal(Vector const & goal)
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  getGoal(Vector & goal) const
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  getActual(Vector & actual) const
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  setGains(Vector const & kp, Vector const & kd)
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  getGains(Vector & kp, Vector & kd) const
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  latch(Model const & model)
  {
    return Status(false, "implement me!");
  }
  
  
  Status TaskPostureController::
  computeCommand(Model const & model, Vector & tau)
  {
    return Status(false, "implement me!");
  }
  
  
 Status TaskPostureController::
  setCycleTime(double dt_seconds)
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  setEndEffector(taoDNode const * end_effector)
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  setControlPoint(Vector const & controlPoint)
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  getControlPoint(Vector & controlPoint) const
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  setMaxvel(Vector const & maxvel)
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  getMaxvel(Vector & maxvel) const
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  setMaxacc(Vector const & maxacc)
  {
    return Status(false, "implement me!");
  }
  
 Status TaskPostureController::
  getMaxacc(Vector & maxacc) const
  {
    return Status(false, "implement me!");
  }
  
    
  TaskPostureController::level_s::
  level_s()
    : cursor(0)
  {
  }
  
    
  TaskPostureController::level_s::
  ~level_s()
  {
    delete cursor;
  }
  
  
  void TaskPostureController::level_s::
  init(size_t ndof, double dt_seconds)
  {
    cursor = new TypeIOTGCursor(ndof, dt_seconds);
    goal = Vector::Zero(ndof);
    maxvel = Vector::Zero(ndof);
    maxacc = Vector::Zero(ndof);
    kp = Vector::Zero(ndof);
    kd = Vector::Zero(ndof);
    goal_changed = false;
  }
  
}
