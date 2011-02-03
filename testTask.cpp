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

#include <opspace/Task.hpp>
#include <opspace/opspace.hpp>
#include <jspace/test/model_library.hpp>
#include <err.h>

using namespace jspace;
using namespace opspace;
using namespace std;

namespace {
  
  class SelectedJointPostureTask : public Task {
  public:
    SelectedJointPostureTask()
      : Task("foo"),
	kp_(100.0),
	kd_(20.0),
	initialized_(false)
    {
      declareParameter("selection", &selection_);
      declareParameter("kp", &kp_);
      declareParameter("kd", &kd_);
    }
    
    virtual Status init(Model const & model) {
      for (size_t ii(0); ii < selection_.rows(); ++ii) {
	if (selection_[ii] > 0.5) {
	  active_joints_.push_back(ii);
	}
      }
      if (active_joints_.empty()) {
	return Status(false, "no active joints");
      }
      actual_ = Vector::Zero(active_joints_.size());
      command_ = Vector::Zero(active_joints_.size());
      jacobian_ = Matrix::Zero(active_joints_.size(), model.getState().position_.rows());
      for (size_t ii(0); ii < active_joints_.size(); ++ii) {
	actual_[ii] = model.getState().position_[active_joints_[ii]];
	jacobian_.coeffRef(ii, active_joints_[ii]) = 1.0;
      }
      initialized_ = true;
      Status ok;
      return ok;
    }
    
    virtual Status update(Model const & model) { 
      Status st;
      if ( ! initialized_) {
	st.ok = false;
	st.errstr = "not initialized";
	return st;
      }
      Vector vel(actual_.rows());
      for (size_t ii(0); ii < active_joints_.size(); ++ii) {
	actual_[ii] = model.getState().position_[active_joints_[ii]];
	vel[ii] = model.getState().velocity_[active_joints_[ii]];
      }
      command_ = -kp_ * actual_ - kd_ * vel;
      return st;
    }
    
    virtual Status check(double const * param, double value) const
    {
      Status st;
      if (((&kp_ == param) && (value < 0)) || ((&kd_ == param) && (value < 0))) {
	st.ok = false;
	st.errstr = "gains must be >= 0";
      }
      return st;
    }
    
    virtual Status check(Vector const * param, Vector const & value) const
    {
      Status st;
      if ((&selection_ == param) && (value.rows() == 0)) {
	st.ok = false;
	st.errstr = "selection must not be empty";
      }
      return st;
    }
    
  protected:
    Vector selection_;
    double kp_;
    double kd_;
    bool initialized_;
    std::vector<size_t> active_joints_;
  };
  
}

int main(int argc, char ** argv)
{
  Model * puma(0);
  
  try {
    warnx("creating Puma model");
    puma = test::create_puma_model();
    size_t const ndof(puma->getNDOF());
    State state(ndof, ndof, 0);
    for (size_t ii(0); ii < ndof; ++ii) {
      state.position_[ii] = 0.1 * ii + 0.8;
      state.velocity_[ii] = 0.2 - 0.05 * ii;
    }
    puma->update(state);
    
    warnx("creating odd SelectedJointPostureTask");
    SelectedJointPostureTask odd;
    Status st;
    odd.dump(cout, "freshly created odd task", "  ");
    
    warnx("testing update before init");
    st = odd.update(*puma);
    if (st) {
      throw runtime_error("odd task update() should have failed before init");
    }
    
    warnx("testing init before selection setting");
    st = odd.init(*puma);
    if (st) {
      throw runtime_error("odd task init() should have failed before setting selection");
    }
    
    warnx("retrieving selection parameter");
    Parameter * selection(odd.lookupParameter("selection", TASK_PARAM_TYPE_VECTOR));
    if ( ! selection) {
      throw runtime_error("failed to retrieve selection parameter");
    }
    
    warnx("trying to set selection");
    Vector sel(Vector::Zero(ndof));
    for (size_t ii(0); ii < ndof; ii += 2) {
      sel[ii] = 1.0;
    }
    st = selection->set(sel);
    if ( ! st) {
      throw runtime_error("failed to set selection: " + st.errstr);
    }
    odd.dump(cout, "odd task after setting selection", "  ");
    
    warnx("testing init");
    st = odd.init(*puma);
    if ( ! st) {
      throw runtime_error("odd task init() failed: " + st.errstr);
    }
    odd.dump(cout, "freshly initialized odd task", "  ");
    
    warnx("testing update");
    st = odd.update(*puma);
    if ( ! st) {
      throw runtime_error("odd task update() failed: " + st.errstr);
    }
    odd.dump(cout, "odd task after update", "  ");
    
    warnx("creating, initializing, and updating even task");
    SelectedJointPostureTask even;
    selection = even.lookupParameter("selection", TASK_PARAM_TYPE_VECTOR);
    sel = Vector::Zero(ndof);
    for (size_t ii(1); ii < ndof; ii += 2) {
      sel[ii] = 1.0;
    }
    selection->set(sel);
    even.init(*puma);
    even.update(*puma);
    even.dump(cout, "even task after update", "  ");
    
    warnx("creating TaskAccumulator");
    Matrix invA;
    if ( ! puma->getInverseMassInertia(invA)) {
      throw runtime_error("failed to get inv mass inertia");
    }
    TaskAccumulator tacc(invA);
    tacc.addTask(odd.getCommand(), odd.getJacobian());
    tacc.addTask(even.getCommand(), even.getJacobian());
    
    cout << "task matrices:\n";
    for (size_t ii(0); ii < tacc.getNLevels(); ++ii) {
      std::ostringstream os;
      os << ii;
      pretty_print(*tacc.getLambda(ii), cout, "  lambda of level " + os.str(), "    ");
      pretty_print(*tacc.getJstar(ii), cout, "  jstar of level " + os.str(), "    ");
      pretty_print(*tacc.getNullspace(ii), cout, "  nullspace of level " + os.str(), "    ");
    }
    pretty_print(tacc.getFinalCommand(), cout, "final command", "  ");
  }
  
  catch (runtime_error const & ee) {
    delete puma;
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  warnx("done with all tests");
  delete puma;
}
