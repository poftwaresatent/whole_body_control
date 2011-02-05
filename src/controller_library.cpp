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
#include <opspace/opspace.hpp>
#include <reflexxes_otg/TypeIOTG.h>
#include <jspace/Model.hpp>
#include <tao/dynamics/taoDNode.h>
#include <tao/dynamics/taoJoint.h>

namespace opspace {

  using jspace::Matrix;
  
  
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
    if (0 > dt_seconds_) {
      return Status(false, "you did not setCycleTime()");
    }
    if (0 == end_effector_) {
      return Status(false, "you did not setEndEffector()");
    }
    if (3 != control_point_.rows()) {
      control_point_ = Vector::Zero(3);
    }
    if (0 == task_.maxvel.rows()) {
      return Status(false, "you did not setMaxvel()");
    }
    if (0 == task_.maxacc.rows()) {
      return Status(false, "you did not setMaxacc()");
    }
    if (0 == task_.kp.rows()) {
      return Status(false, "you did not setGains()");
    }
    
    size_t const ndof(model.getNDOF());
    if (ndof != posture_.maxvel.rows()) {
      return Status(false, "invalid maxvel dimension");
    }
    if (ndof != posture_.maxacc.rows()) {
      return Status(false, "invalid maxacc dimension");
    }
    if (ndof != posture_.kp.rows()) {
      return Status(false, "invalid kp dimension");
    }
    if (ndof != posture_.kd.rows()) {
      return Status(false, "invalid kd dimension");
    }
    
    posture_.cursor = new TypeIOTGCursor(ndof, dt_seconds_);
    task_.cursor = new TypeIOTGCursor(3, dt_seconds_);
    
    return latch(model);
  }
  
  
  Status TaskPostureController::
  setGoal(Vector const & goal)
  {
    Status st;
    if (3 >= goal.rows()) {
      st.ok = false;
      st.errstr = "invalid goal dimension";
      return st;
    }
    task_.goal = goal.block(0, 0, 3, 1);
    task_.goal_changed = true;	// here we cannot actually distinguish
				// task from posture goal
				// changes... ah well, that'll be
				// easier with generic TaskParameter
				// instances.
    posture_.goal = goal.block(3, 0, goal.rows() - 3, 1);
    posture_.goal_changed = true;
    return st;
  }
  
  
  Status TaskPostureController::
  getGoal(Vector & goal) const
  {
    if (0 == task_.goal.rows()) {
      return Status(false, "undefined goal, call setGoal() first");
    }
    goal.resize(task_.goal.rows() + posture_.goal.rows());
    goal << task_.goal, posture_.goal;
    Status ok;
    return ok;
  }
  
  
  Status TaskPostureController::
  getActual(Vector & actual) const
  {
    if (0 == actual_.rows()) {
      return Status(false, "undefined actual state, call init() first");
    }
    actual = actual_;
    Status ok;
    return ok;
  }
  
  
  Status TaskPostureController::
  setGains(Vector const & kp, Vector const & kd)
  {
    Status st;
    if ((3 >= kp.rows()) || (3 >= kd.rows())) {
      st.ok = false;
      st.errstr = "invalid gain dimension";
      return st;
    }
    task_.kp = kp.block(0, 0, 3, 1);
    task_.kd = kd.block(0, 0, 3, 1);
    posture_.kp = kp.block(3, 0, kp.rows() - 3, 1);
    posture_.kd = kd.block(3, 0, kd.rows() - 3, 1);
    return st;
  }
  
  
  Status TaskPostureController::
  getGains(Vector & kp, Vector & kd) const
  {
    if (0 == task_.kp.rows()) {
      return Status(false, "undefined gains, call setGains() first");
    }
    kp.resize(task_.kp.rows() + posture_.kp.rows());
    kp << task_.kp, posture_.kp;
    kd.resize(task_.kd.rows() + posture_.kd.rows());
    kd << task_.kd, posture_.kd;
    Status ok;
    return ok;
  }
  
  
  Status TaskPostureController::
  latch(Model const & model)
  {
    Status st(updateActual(model));
    if ( ! st) {
      return st;
    }
    posture_.goal = model.getState().position_;
    posture_.goal_changed = true;
    task_.goal = actual_.block(0, 0, 3, 1);
    task_.goal_changed = true;
    return st;
  }
  
  
  Status TaskPostureController::
  computeCommand(Model const & model, Vector & tau)
  {
    Status st(updateActual(model));
    if ( ! st) {
      return st;
    }
    
    //////////////////////////////////////////////////
    // end-effector Cartesian position task
    
    Matrix Jfull;
    if ( ! model.computeJacobian(end_effector_,
				 actual_[0],
				 actual_[1],
				 actual_[2],
				 Jfull)) {
      return Status(false, "failed to compute Jacobian (probably caused by an unsupported joint type)");
    }
    
    // There must be a way to store just a "block" object that refers
    // to Jfull but acts like Jx, without copying the actual
    // elements... but the eigen2 documentation did not show me the
    // way to do it.
    size_t const ndof(model.getNDOF());
    Matrix Jx(Jfull.block(0, 0, 3, ndof));
    
    Matrix invA;
    if ( ! model.getInverseMassInertia(invA)) {
      return Status(false, "failed to read inverse mass inertia (wow, how did that happen?)");
    }
    Matrix Lambda_t;
    pseudoInverse(Jx * invA * Jx.transpose(), 1e-3, Lambda_t);

    // Here, too, it would be nice to just store a reference (see
    // comments on Jx above).
    Vector curpos(actual_.block(0, 0, 3, 1));
    Vector curvel(Jx * model.getState().velocity_);
    if (task_.goal_changed) {
      task_.cursor->position() = curpos;
      task_.cursor->velocity() = curvel;
      task_.goal_changed = false;
    }
    int otg_result(task_.cursor->next(task_.maxvel,
				      task_.maxacc,
				      task_.goal));
    if (0 > otg_result) {
      return Status(false, "OTG error during task trajectory generation");
    }
    Vector tau_task(Jx.transpose() * (-Lambda_t)
		    * (   task_.kp.cwise() * (curpos - task_.cursor->position())
		        + task_.kd.cwise() * (curvel - task_.cursor->velocity())));
    
    //////////////////////////////////////////////////
    // joint posture projected into nullspace of task
    
    Matrix Jbar(invA * Jx.transpose() * Lambda_t);
    Matrix nullspace(Matrix::Identity(ndof, ndof) - Jbar * Jx);
    Matrix Lambda_p;
    pseudoInverse(nullspace * invA, 1e-3, Lambda_p);
    
    if (posture_.goal_changed) {
      posture_.cursor->position() = model.getState().position_;
      posture_.cursor->velocity() = model.getState().velocity_;
      posture_.goal_changed = false;
    }
    otg_result = posture_.cursor->next(posture_.maxvel,
				       posture_.maxacc,
				       posture_.goal);
    if (0 > otg_result) {
      return Status(false, "OTG error during posture trajectory generation");
    }
    Vector tau_posture(nullspace.transpose() * (-Lambda_p)
		       * (  posture_.kp.cwise() * (model.getState().position_ - posture_.cursor->position())
			  + posture_.kd.cwise() * (model.getState().velocity_ - posture_.cursor->velocity())));
    
    //////////////////////////////////////////////////
    // sum it up with gravity...
    
    Vector gg;
    if ( ! model.getGravity(gg)) {
      return Status(false, "failed to read gravity torque (wow, how did that happen?)");
    }
    tau = tau_task + tau_posture + gg;
    
    return st;
  }
  
  
  Status TaskPostureController::
  setCycleTime(double dt_seconds)
  {
    Status st;
    if (0 >= dt_seconds) {
      st.ok = false;
      st.errstr = "cycle time must be > 0";
      return st;
    }
    dt_seconds_ = dt_seconds;
    return st;
  }
  
  
  Status TaskPostureController::
  setEndEffector(taoDNode const * end_effector)
  {
    Status st;
    if (0 == end_effector) {
      st.ok = false;
      st.errstr = "invalid end effector";
      return st;
    }
    end_effector_ = end_effector;
    return st;
  }
  
  
  Status TaskPostureController::
  setControlPoint(Vector const & control_point)
  {
    Status st;
    if (3 != control_point.rows()) {
      st.ok = false;
      st.errstr = "invalid control point dimension";
      return st;
    }
    control_point_ = control_point;
    return st;
  }
  
  
  Status TaskPostureController::
  getControlPoint(Vector & control_point) const
  {
    if (0 == control_point_.rows()) {
      control_point_ = Vector::Zero(3);
    }
    control_point = control_point_;
    Status ok;
    return ok;
  }
  
  
  Status TaskPostureController::
  setMaxvel(Vector const & maxvel)
  {
    Status st;
    if (3 >= maxvel.rows()) {
      st.ok = false;
      st.errstr = "invalid maxvel dimension";
      return st;
    }
    task_.maxvel = maxvel.block(0, 0, 3, 1);
    posture_.maxvel = maxvel.block(3, 0, maxvel.rows() - 3, 1);
    return st;
  }
  
  
  Status TaskPostureController::
  getMaxvel(Vector & maxvel) const
  {
    if (0 == task_.maxvel.rows()) {
      return Status(false, "undefined maxvel, call setMaxvel() first");
    }
    maxvel.resize(task_.maxvel.rows() + posture_.maxvel.rows());
    maxvel << task_.maxvel, posture_.maxvel;
    Status ok;
    return ok;
  }
  
  
  Status TaskPostureController::
  setMaxacc(Vector const & maxacc)
  {
    Status st;
    if (3 >= maxacc.rows()) {
      st.ok = false;
      st.errstr = "invalid maxacc dimension";
      return st;
    }
    task_.maxacc = maxacc.block(0, 0, 3, 1);
    posture_.maxacc = maxacc.block(3, 0, maxacc.rows() - 3, 1);
    return st;
  }
  
  
  Status TaskPostureController::
  getMaxacc(Vector & maxacc) const
  {
    if (0 == task_.maxacc.rows()) {
      return Status(false, "undefined maxacc, call setMaxacc() first");
    }
    maxacc.resize(task_.maxacc.rows() + posture_.maxacc.rows());
    maxacc << task_.maxacc, posture_.maxacc;
    Status ok;
    return ok;
  }
  
  
  Status TaskPostureController::
  updateActual(Model const & model)
  {
    if (0 == end_effector_) {
      return Status(false, "you did not call setEndEffector()");
    }
    if (0 == control_point_.rows()) {
      control_point_ = Vector::Zero(3);
    }
    jspace::Transform ee;
    if ( ! model.computeGlobalFrame(end_effector_,
				    control_point_[0],
				    control_point_[1],
				    control_point_[2],
				    ee)) {
      return Status(false, "you gave me an invalid end effector");
    }
    actual_.resize(3 + model.getNDOF());
    actual_ << ee.translation(), model.getState().position_;
    Status ok;
    return ok;
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
  
}
