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

#include <TaskDescription.hpp>

using namespace opspace;

namespace {
  
  class TaskFoo : public TaskDescription {
  public:
    TaskFoo():
      TaskDescription("foo", TASK_PARAM_SELECT_ALL),
      initialized_(false)
    {
      foo_ = defineParameter("foo", TASK_PARAM_TYPE_REAL);
      *(foo_->getReal()) = 42.17;
    }
    
    virtual Status init(Model const & model) {
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
      command_ = Vector::Zero(actual_.cols());
      Jacobian_ = Matrix::Identity(actual_.cols(), actual_.cols());
      return st;
    }

  protected:
    bool initialized_;
    TaskParameterEntry * foo_;
  };
  
}

int main(int argc, char ** argv)
{
  TaskFoo foo;
}
