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

#include <opspace/ControllerNG.hpp>
#include <opspace/opspace.hpp>

// hmm...
#include <Eigen/LU>
#include <Eigen/SVD>
#include <opspace/task_library.hpp>

using jspace::pretty_print;
using boost::shared_ptr;

namespace opspace {
  
  
  ControllerNG::
  ControllerNG(std::string const & name)
    : name_(name),
      fallback_(false),
      loglen_(-1),
      logsubsample_(-1),
      logprefix_(""),
      logcount_(-1)
  {
    declareParameter("loglen", &loglen_, PARAMETER_FLAG_NOLOG);
    declareParameter("logsubsample", &logsubsample_, PARAMETER_FLAG_NOLOG);
    declareParameter("logprefix", &logprefix_, PARAMETER_FLAG_NOLOG);
    declareParameter("jpos", &jpos_);
    declareParameter("jvel", &jvel_);
    declareParameter("gamma", &gamma_);
  }
  
  
  Status ControllerNG::
  check(std::string const * param, std::string const & value) const
  {
    if ((param == &logprefix_) && (loglen_ > 0)) {
      // could be smart and check if another log is ongoing, but let's
      // just abort that
      logcount_ = 0;
    }
    Status ok;
    return ok;
  }
  
  
  void ControllerNG::
  setFallbackTask(shared_ptr<Task> task)
  {
    fallback_task_ = task;
  }
  
  
  Status ControllerNG::
  init(Model const & model)
  {
    if ( ! fallback_task_) {
      JPosTrjTask * pt(new JPosTrjTask("ControllerNG_fallback_posture"));
      pt->quickSetup(0.01, 50, 5, 1, 2);
      fallback_task_.reset(pt);
    }
    if ( ! dynamic_cast<JPosTrjTask*>(fallback_task_.get())) {
      return Status(false, "fallback task has to be a posture (for now)");
    }
    return Status();
  }
  
  
  Status ControllerNG::
  computeFallback(Model const & model,
		  bool init_required,
		  Vector & gamma)
  {
    Status st;
    
    if (init_required) {
      st = fallback_task_->init(model);
      if ( ! st) {
	return Status(false, "fallback task failed to initialize: " + st.errstr);
      }
    }
    
    st = fallback_task_->update(model);
    if ( ! st) {
      return Status(false, "fallback task failed to update: " + st.errstr);
    }
    
    Matrix aa;
    if ( ! model.getMassInertia(aa)) {
      return Status(false, "failed to retrieve mass inertia");
    }
    Vector grav;
    if ( ! model.getGravity(grav)) {
      return Status(false, "failed to retrieve gravity torques");
    }
    gamma = aa * fallback_task_->getCommand() + grav;
    
    // logging and debug
    jpos_ = model.getState().position_;
    jvel_ = model.getState().velocity_;
    gamma_ = gamma;
    
    return st;
  }
  
  
  Status ControllerNG::
  computeCommand(Model const & model,
		 Behavior & behavior,
		 Vector & gamma)
  {
    //////////////////////////////////////////////////
    // Shortcut if we are already in fallback mode. Later, we'll have
    // to code a way to get out of fallback, but for now it's a
    // one-way ticket.
    if (fallback_) {
      return computeFallback(model, false, gamma);
    }
    //////////////////////////////////////////////////
    
    Status st(behavior.update(model));
    if ( ! st) {
      fallback_ = true;
      fallback_reason_ = "behavior update failed: " + st.errstr;
      return computeFallback(model, true, gamma);
    }
    
    Behavior::task_table_t const * tasks(behavior.getTaskTable());
    if ( ! tasks) {
      fallback_ = true;
      fallback_reason_ = "null task table";
      return computeFallback(model, true, gamma);
    }
    if (tasks->empty()) {
      fallback_ = true;
      fallback_reason_ = "empty task table";
      return computeFallback(model, true, gamma);
    }
    
    //////////////////////////////////////////////////
    // the magic nullspace sauce...
    
    Matrix ainv;
    if ( ! model.getInverseMassInertia(ainv)) {
      return Status(false, "failed to retrieve inverse mass inertia");
    }
    Vector grav;
    if ( ! model.getGravity(grav)) {
      return Status(false, "failed to retrieve gravity torques");
    }
    
    size_t const ndof(model.getNDOF());
    size_t const n_minus_1(tasks->size() - 1);
    Matrix nstar(Matrix::Identity(ndof, ndof));
    int first_active_task_index(0); // because tasks can have empty Jacobian
    
    ////    sv_lstar_.resize(tasks->size());
    if (sv_jstar_.empty()) {
      //##################################################
      //##################################################
      //##################################################
      //##################################################
      // quick hack, will break as soon as we switch behavior at
      // runtime, but there's no duplicate-detection behind
      // declareParameter() so it's a bit tricky to declare them on
      // the fly...
      //##################################################
      //##################################################
      //##################################################
      //##################################################
      sv_jstar_.resize(tasks->size());
      for (size_t ii(0); ii < tasks->size(); ++ii) {
	declareParameter("sv_jstar-" + (*tasks)[ii]->getName(), &(sv_jstar_[ii]));
      }
    }
    
    for (size_t ii(0); ii < tasks->size(); ++ii) {
      
      Task const * task((*tasks)[ii]);
      Matrix const & jac(task->getJacobian());
      
      // skip inactive tasks at beginning of table
      if ((0 == jac.rows()) || (0 == jac.cols())) {
	++first_active_task_index;
	////	sv_lstar_[ii].resize(0);
	sv_jstar_[ii].resize(0);
	continue;
      }
      
      Matrix jstar;
      if (ii == first_active_task_index) {
	jstar = jac;
      }
      else {
	jstar = jac * nstar;
      }
      
      Matrix jjt(jstar * jstar.transpose());
      sv_jstar_[ii] = Eigen::SVD<Matrix>(jjt).singularValues();
      st = behavior.checkJStarSV(task, sv_jstar_[ii]);
      if ( ! st) {
	fallback_ = true;
	fallback_reason_ = "checkJStarSV failed: " + st.errstr;
	return computeFallback(model, true, gamma);
      }
      
      Matrix lstar;
      pseudoInverse(jstar * ainv * jstar.transpose(),
		    task->getSigmaThreshold(),
		    lstar, 0);////&sv_lstar_[ii]);
      Vector pstar;
      pstar = lstar * jstar * ainv * grav; // same would go for coriolis-centrifugal...
      
      // could add coriolis-centrifugal just like pstar...
      if (ii == first_active_task_index) {
	// first time around: initialize gamma
	gamma = jstar.transpose() * (lstar * task->getCommand() + pstar);
      }
      else {
	Vector fcomp;
	// here, gamma is still at the previous iteration's value
	fcomp = lstar * jstar * ainv * gamma;
	gamma += jstar.transpose() * (lstar * task->getCommand() + pstar - fcomp);
      }
      
      if (ii != n_minus_1) {
	// not sure whether Eigen2 correctly handles the case where a
	// matrix gets updated by left-multiplication...
	Matrix const
	  nnext((Matrix::Identity(ndof, ndof) - ainv * jstar.transpose() * lstar * jstar) * nstar);
	nstar = nnext;
      }
    }
    
    if (tasks->size() <= first_active_task_index) {
      fallback_ = true;
      fallback_reason_ = "no active tasks";
      return computeFallback(model, true, gamma);
    }
    
    // logging and debug
    jpos_ = model.getState().position_;
    jvel_ = model.getState().velocity_;
    gamma_ = gamma;
    
    return st;
  }
  
  
  void ControllerNG::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "log count: " << logcount_ << "\n"
       << prefix << "parameters\n";
    dump(os, "", prefix + "  ");
    if (fallback_) {
      os << prefix << "# FALLBACK MODE ENABLED ##########################\n"
	 << prefix << "# reason: " << fallback_reason_ << "\n";
    }
  }
  
  
  void ControllerNG::
  qhlog(Behavior & behavior, long long timestamp)
  {
    if (0 == logcount_) {
      // initialize logging
      log_.clear();
      log_.push_back(shared_ptr<ParameterLog>(new ParameterLog("ctrl_" + name_,
							       getParameterTable())));
      log_.push_back(shared_ptr<ParameterLog>(new ParameterLog("skill_" + behavior.getName(),
							       behavior.getParameterTable())));
      Behavior::task_table_t const * tasks(behavior.getTaskTable());
      if (tasks) {
	for (size_t ii(0); ii < tasks->size(); ++ii) {
	  std::ostringstream nm;
	  nm << "task_" << ii << "_" << (*tasks)[ii]->getName();
	  log_.push_back(shared_ptr<ParameterLog>(new ParameterLog(nm.str(),
								   (*tasks)[ii]->getParameterTable())));
	}
      }
    }
    else if ((0 < loglen_) && (loglen_ == logcount_)) {
      logcount_ = -2;
    }
    if (0 <= logcount_) {
      ++logcount_;
    }
    
    if (0 < logcount_) {
      if ((logsubsample_ <= 0)
	  || (0 == (logcount_ % logsubsample_))) {
	for (size_t ii(0); ii < log_.size(); ++ii) {
	  log_[ii]->update(timestamp);
	}
      }
    }
    
    if (-2 == logcount_) {
      for (size_t ii(0); ii < log_.size(); ++ii) {
	log_[ii]->writeFiles(logprefix_, &std::cerr);
      }
      logcount_ = -1;
    }
  }
  
}
