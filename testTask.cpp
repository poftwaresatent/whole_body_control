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

#include <Task.hpp>
#include <jspace/test/model_library.hpp>
#include <err.h>

using namespace jspace;
using namespace opspace;
using namespace std;

namespace {
  
  class TaskFoo : public Task {
  public:
    TaskFoo()
      : Task("foo", TASK_PARAM_SELECT_ALL),
	initialized_(false)
    {
      foo_ = defineParameter("foo", TASK_PARAM_TYPE_REAL);
      foo_->setReal(42.17);
    }
    
    virtual Status init(Model const & model) {
      goal_->initVector(model.getState().position_);
      actual_ = model.getState().position_;
      initialized_ = true;
      return update(model);
    }
    
    virtual Status update(Model const & model) { 
      Status st;
      if ( ! initialized_) {
	st.ok = false;
	st.errstr = "not initialized";
	return st;
      }
      actual_ = model.getState().position_;
      command_ = Vector::Zero(actual_.rows());
      Jacobian_ = Matrix::Identity(actual_.rows(), actual_.rows());
      return st;
    }
    
    virtual Status checkVector(TaskParameter const * param, Vector const & value) const
    {
      Status st;
      if (param == goal_) {
	if (value.rows() != actual_.rows()) {
	  st.ok = false;
	  st.errstr = "invalid goal dimension";
	  return st;
	}
      }
      return st;
    }
    
  protected:
    bool initialized_;
    TaskParameter * foo_;
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
    
    warnx("creating TaskFoo");
    TaskFoo foo;
    Status st;
    foo.dump(cout, "freshly created TaskFoo", "  ");
    
    warnx("testing update before init");
    st = foo.update(*puma);
    if (st) {
      throw runtime_error("TaskFoo::update() should have failed before init");
    }
    
    warnx("testing init");
    st = foo.init(*puma);
    if ( ! st) {
      throw runtime_error("TaskFoo::init() failed: " + st.errstr);
    }
    foo.dump(cout, "freshly initialized TaskFoo", "  ");
    
    warnx("testing update");
    st = foo.update(*puma);
    if ( ! st) {
      throw runtime_error("TaskFoo::update() failed: " + st.errstr);
    }
    foo.dump(cout, "TaskFoo after update", "  ");
    
    warnx("retrieving goal parameter");
    TaskParameter * goal(foo.lookupParameter("goal", TASK_PARAM_TYPE_VECTOR));
    if ( ! goal) {
      throw runtime_error("failed to retrieve goal parameter");
    }
    
    warnx("trying to set invalid goal");
    st = goal->setVector(Vector::Ones(ndof + 1));
    if (st) {
      throw runtime_error("failed to fail on invalid goal");
    }
    
    warnx("trying to set valid goal");
    st = goal->setVector(Vector::Ones(ndof));
    if ( ! st) {
      throw runtime_error("failed to set valid goal");
    }
    foo.dump(cout, "TaskFoo after setting goal", "  ");
  }
  
  catch (runtime_error const & ee) {
    delete puma;
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  warnx("done with all tests");
  delete puma;
}
