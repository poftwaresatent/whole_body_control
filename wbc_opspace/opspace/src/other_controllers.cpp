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

#include <opspace/other_controllers.hpp>
#include <opspace/opspace.hpp>

// hmm...
#include <Eigen/LU>
#include <Eigen/SVD>

using jspace::pretty_print;

namespace opspace {


  SController::
  SController(std::string const & name, std::ostream * dbg)
    : Controller(name, dbg)
  {
  }
  
  
  Status SController::
  computeCommand(Model const & model, Vector & gamma)
  {
    if ( ! initialized_) {
      return Status(false, "not initialized");
    }
    Matrix ainv;
    if ( ! model.getInverseMassInertia(ainv)) {
      return Status(false, "failed to retrieve inverse mass inertia");
    }
    Vector grav;
    if ( ! model.getGravity(grav)) {
      return Status(false, "failed to retrieve gravity torques");
    }
    
    std::ostringstream msg;
    bool ok(true);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Task * task(task_table_[ii].get());
      Status const st(task->update(model));
      if ( ! st) {
	msg << "  task[" << ii << "] `" << task->getName() << "': " << st.errstr << "\n";
	ok = false;
      }
    }
    if ( ! ok) {
      return Status(false, "failures during task updates:\n" + msg.str());
    }
    
    if (dbg_) {
      *dbg_ << "DEBUG opspace::SController::computeCommand\n";
      pretty_print(model.getState().position_, *dbg_, "  jpos", "    ");
      pretty_print(model.getState().velocity_, *dbg_, "  jvel", "    ");
      pretty_print(grav, *dbg_, "  grav", "    ");
      pretty_print(ainv, *dbg_, "  ainv", "    ");
    }
    
    Matrix rangespace;
    size_t const ndof(model.getNDOF());
    size_t const n_minus_1(task_table_.size() - 1);
    
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      
      Task const * task(task_table_[ii].get());
      Matrix const & jac(task->getJacobian());
      
      Matrix lambda, jbar;
      Vector tau;
      
      pseudoInverse(jac * ainv * jac.transpose(), 1e-3, lambda);

      jbar = ainv * jac.transpose() * lambda;
      Matrix const jtjbt(jac.transpose() * jbar.transpose());
      tau = jac.transpose() * lambda * task->getCommand() + jtjbt * grav;
      
      if (dbg_) {
	Vector const taunog(tau - jtjbt * grav);
	*dbg_ << "  task [" << ii << "] " << task->getName() << "\n";
	pretty_print(jac,    *dbg_, "    jac", "      ");
	pretty_print(lambda, *dbg_, "    lambda", "      ");
	pretty_print(jbar,   *dbg_, "    jbar", "      ");
	pretty_print(jtjbt,  *dbg_, "    jac.t * jbar.t", "      ");
	pretty_print(taunog, *dbg_, "    tau w/o gravity", "      ");
	pretty_print(tau,    *dbg_, "    tau", "      ");
      }
      
      if (0 == ii) {
	gamma = tau;
	rangespace = jtjbt;	// for next task
      }
      else {
	Matrix const nullspace(Matrix::Identity(ndof, ndof) - rangespace);
	gamma += nullspace * tau;
	if (ii != n_minus_1) {
	  rangespace *= jtjbt;	// for next task
	}
	if (dbg_) {
	  Vector const tauproj(nullspace * tau);
	  pretty_print(nullspace, *dbg_, "    nullspace", "      ");
	  pretty_print(tauproj,   *dbg_, "    tau projected", "      ");
	}
      }
    }
    
    if (dbg_) {
      pretty_print(gamma, *dbg_, "  gamma", "    ");
    }
    
    Status st;
    return st;
  }
  
  
  LRController::
  LRController(std::string const & name, std::ostream * dbg)
    : Controller(name, dbg)
  {
  }
  
  
  Status LRController::
  computeCommand(Model const & model, Vector & gamma)
  {
    if ( ! initialized_) {
      return Status(false, "not initialized");
    }
    Matrix ainv;
    if ( ! model.getInverseMassInertia(ainv)) {
      return Status(false, "failed to retrieve inverse mass inertia");
    }
    Vector grav;
    if ( ! model.getGravity(grav)) {
      return Status(false, "failed to retrieve gravity torques");
    }
    
    std::ostringstream msg;
    bool ok(true);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Task * task(task_table_[ii].get());
      Status const st(task->update(model));
      if ( ! st) {
	msg << "  task[" << ii << "] `" << task->getName() << "': " << st.errstr << "\n";
	ok = false;
      }
    }
    if ( ! ok) {
      return Status(false, "failures during task updates:\n" + msg.str());
    }
    
    if (dbg_) {
      *dbg_ << "DEBUG opspace::LRController::computeCommand\n";
      pretty_print(model.getState().position_, *dbg_, "  jpos", "    ");
      pretty_print(model.getState().velocity_, *dbg_, "  jvel", "    ");
      pretty_print(grav, *dbg_, "  grav", "    ");
      pretty_print(ainv, *dbg_, "  ainv", "    ");
    }
    
    size_t const ndof(model.getNDOF());
    size_t const n_minus_1(task_table_.size() - 1);
    Matrix nstar(Matrix::Identity(ndof, ndof));
    int first_active_task_index(0); // because tasks can have empty Jacobian
    
    sv_lstar_.resize(task_table_.size());
    sv_jstar_.resize(task_table_.size());
    
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      
      Task const * task(task_table_[ii].get());
      Matrix const & jac(task->getJacobian());
      
      if (dbg_) {
	*dbg_ << "  task [" << ii << "] " << task->getName() << "\n";
	pretty_print(jac, *dbg_, "    jac", "      ");
      }
      
      if ((0 == jac.rows()) || (0 == jac.cols())) {
	++first_active_task_index; // in case the first few tasks are inactive
	if (dbg_) {
	  *dbg_ << "    jacobian is empty, skip this task\n";
	}
	sv_lstar_[ii].resize(0);
	sv_jstar_[ii].resize(0);
	continue;
      }
      
      Matrix jstar;
      if (ii == first_active_task_index) {
	jstar = jac;
      }
      else {
	jstar = jac * nstar;
	if (dbg_) {
	  pretty_print(nstar, *dbg_, "    nstar", "      ");
	  pretty_print(jstar,     *dbg_, "    jstar", "      ");
	}
      }
      
      {
	// debugging the singularity problem
	Matrix jjt(jstar * jstar.transpose());
	sv_jstar_[ii] = Eigen::SVD<Matrix>(jjt).singularValues();
      }
      
      Matrix lstar;
      {
	// try regularized inverse
	Matrix reg(jstar * ainv * jstar.transpose()
		   + task->getSigmaThreshold() * Matrix::Identity(jstar.rows(), jstar.rows()));
	lstar = reg.inverse();
	// still debug lambda eigenvalues
	sv_lstar_[ii] = Eigen::SVD<Matrix>(lstar).singularValues();
      }
      
      Vector pstar;
      pstar = lstar * jstar * ainv * grav; // same would go for coriolis-centrifugal...
      
      if (dbg_) {
	pretty_print(lstar, *dbg_, "    lstar", "      ");
	pretty_print(pstar, *dbg_, "    pstar", "      ");
      }
      
      // could add coriolis-centrifugal just like pstar...
      if (ii == first_active_task_index) {
	// first time around: initialize gamma
	gamma = jstar.transpose() * (lstar * task->getCommand() + pstar);
	if (dbg_) {
	  Vector const taunog(jstar.transpose() * (lstar * task->getCommand()));
	  Vector const tau(jstar.transpose() * (lstar * task->getCommand() + pstar));
	  pretty_print(taunog, *dbg_, "    tau w/o gravity", "      ");
	  pretty_print(tau,    *dbg_, "    tau", "      ");
	}
      }
      else {
	Vector fcomp;
	// here, gamma is still at the previous iteration's value
	fcomp = lstar * jstar * ainv * gamma;
	gamma += jstar.transpose() * (lstar * task->getCommand() + pstar - fcomp);
	if (dbg_) {
	  Vector const taunog(jstar.transpose() * (lstar * task->getCommand()));
	  Vector const tau(jstar.transpose() * (lstar * task->getCommand() + pstar - fcomp));
	  pretty_print(fcomp,  *dbg_, "    fcomp", "      ");
	  pretty_print(taunog, *dbg_, "    tau w/o (grav, comp)", "      ");
	  pretty_print(tau,    *dbg_, "    tau", "      ");
	}
	
      }
      
      if (ii != n_minus_1) {
	// not sure whether Eigen2 correctly handles the case where a
	// matrix gets updated by left-multiplication...
	Matrix const nnext((Matrix::Identity(ndof, ndof) - ainv * jstar.transpose() * lstar * jstar) * nstar);
	nstar = nnext;
      }
    }
    
    if (dbg_) {
      pretty_print(gamma, *dbg_, "  gamma", "    ");
    }
    
    Status st;
    return st;
  }
  
  
  void LRController::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "Singular values of tasks in LRController: `" << name_ << "'\n";
    for (size_t ii(0); ii < sv_lstar_.size(); ++ii) {
      os << prefix << "  J* " << ii << "\n";
      pretty_print(sv_jstar_[ii], os, "", prefix + "    ");
      os << prefix << "  L* " << ii << "\n";
      pretty_print(sv_lstar_[ii], os, "", prefix + "    ");
    }
  }

}
