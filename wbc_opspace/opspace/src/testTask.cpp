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

#include <gtest/gtest.h>
#include <opspace/task_library.hpp>
#include <opspace/Controller.hpp>
#include <jspace/test/model_library.hpp>
#include <err.h>

using jspace::Model;
using jspace::State;
using jspace::pretty_print;
using namespace opspace;
using namespace std;


static Model * get_puma()
{
  static Model * puma(0);
  if ( ! puma) {
    puma = jspace::test::create_puma_model();
  }
  size_t const ndof(puma->getNDOF());
  State state(ndof, ndof, 0);
  for (size_t ii(0); ii < ndof; ++ii) {
    state.position_[ii] = 0.01 * ii + 0.08;
    state.velocity_[ii] = 0.02 - 0.005 * ii;
  }
  puma->update(state);
  return puma;
}


TEST (task, basics)
{
  Model * puma(get_puma());
  SelectedJointPostureTask odd("odd");
  Status st;
  
  st = odd.update(*puma);
  EXPECT_FALSE (st.ok) << "update before init should have failed";
  
  st = odd.init(*puma);
  EXPECT_FALSE (st.ok) << "init before selection setting should have failed";
  
  Parameter * selection(odd.lookupParameter("selection", TASK_PARAM_TYPE_VECTOR));
  ASSERT_NE ((void*)0, selection) << "failed to retrieve selection parameter";
  
  Vector sel(Vector::Zero(puma->getNDOF()));
  for (size_t ii(0); ii < puma->getNDOF(); ii += 2) {
    sel[ii] = 1.0;
  }
  st = selection->set(sel);
  EXPECT_TRUE (st.ok) << "failed to set selection: " << st.errstr;
  
  st = odd.init(*puma);
  EXPECT_TRUE (st.ok) << "init failed: " << st.errstr;
  
  st = odd.update(*puma);
  EXPECT_TRUE (st.ok) << "update failed: " << st.errstr;
}


static SelectedJointPostureTask * create_sel_jp_task(string const & name, Vector const & selection)
  throw(runtime_error)
{
  SelectedJointPostureTask * task(new SelectedJointPostureTask(name));
  Parameter * sel_p(task->lookupParameter("selection", TASK_PARAM_TYPE_VECTOR));
  if ( ! sel_p) {
    delete task;
    throw runtime_error("failed to retrieve selection parameter");
  }
  Status const st(sel_p->set(selection));
  if ( ! st) {
    delete task;
    throw runtime_error("failed to set selection: " + st.errstr);
  }
  return task;
}


static void append_odd_even_tasks(Controller & ctrl, size_t ndof)
  throw(runtime_error)
{
  vector<Task*> task;
  try {
    Vector sel_odd(Vector::Zero(ndof));
    Vector sel_even(Vector::Zero(ndof));
    for (size_t ii(0); ii < ndof; ++ii) {
      if (0 == (ii % 2)) {
	sel_even[ii] = 1.0;
      }
      else {
	sel_odd[ii] = 1.0;
      }
    }
    task.push_back(create_sel_jp_task("odd", sel_odd));
    task.push_back(create_sel_jp_task("even", sel_even));
    for (size_t ii(0); ii < task.size(); ++ii) {
      if ( ! ctrl.appendTask(task[ii], true)) {
	throw runtime_error("failed to add task `" + task[ii]->getName() + "'");
      }
      task[ii] = 0;	   // avoid double-free in case we throw later
    }
  }
  catch (runtime_error const & ee) {
    for (size_t ii(0); ii < task.size(); ++ii) {
      delete task[ii];
    }
    throw ee;
  }
}


static void append_odd_full_tasks(Controller & ctrl, size_t ndof)
  throw(runtime_error)
{
  vector<Task*> task;
  try {
    Vector sel_odd(Vector::Zero(ndof));
    Vector sel_full(Vector::Ones(ndof));
    for (size_t ii(1); ii < ndof; ii += 2) {
      sel_odd[ii] = 1.0;
    }
    task.push_back(create_sel_jp_task("odd", sel_odd));
    task.push_back(create_sel_jp_task("full", sel_full));
    for (size_t ii(0); ii < task.size(); ++ii) {
      if ( ! ctrl.appendTask(task[ii], true)) {
	throw runtime_error("failed to add task `" + task[ii]->getName() + "'");
      }
      task[ii] = 0;	   // avoid double-free in case we throw later
    }
  }
  catch (runtime_error const & ee) {
    for (size_t ii(0); ii < task.size(); ++ii) {
      delete task[ii];
    }
    throw ee;
  }
}


TEST (controller, odd_even)
{
  Task * jpos(0);
  Vector gamma_jpos;
  
  vector<Controller*> ctrl;
  vector<ostringstream*> msg;
  vector<Vector> gamma;

  try {
    Model * puma(get_puma());
    Matrix aa;
    Vector gg;
    ASSERT_TRUE (puma->getMassInertia(aa)) << "failed to get mass inertia";
    ASSERT_TRUE (puma->getGravity(gg)) << "failed to get gravity";
    
    jpos = create_sel_jp_task("all", Vector::Ones(puma->getNDOF()));
    Status st;
    st = jpos->init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init jpos task: " << st.errstr;
    st = jpos->update(*puma);
    EXPECT_TRUE (st.ok) << "failed to update jpos task: " << st.errstr;
    gamma_jpos = aa * jpos->getCommand() + gg;
    
    msg.push_back(new ostringstream());
    ctrl.push_back(new SController("Samir", msg.back()));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    msg.push_back(new ostringstream());
    ctrl.push_back(new LController("Luis", msg.back()));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      
      append_odd_even_tasks(*ctrl[ii], puma->getNDOF());
      st = ctrl[ii]->init(*puma);
      EXPECT_TRUE (st.ok) << "failed to init controller #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
      st = ctrl[ii]->computeCommand(*puma, gamma[ii]);
      EXPECT_TRUE (st.ok) << "failed to compute torques #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
      
      cout << "==================================================\n"
	   << "messages from controller #" << ii << " `" << ctrl[ii]->getName() << "'\n"
	   << "--------------------------------------------------\n"
	   << msg[ii]->str();
    }
    
    cout << "==================================================\n"
	 << "whole-body torque comparison:\n";
    pretty_print(gamma_jpos, cout, "  reference jpos task", "    ");
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      pretty_print(gamma[ii], cout, "  controller `" + ctrl[ii]->getName() + "'", "    ");
      Vector const delta(gamma_jpos - gamma[ii]);
      pretty_print(delta, cout, "  delta", "    ");
    }
    
  }
  catch (exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      delete ctrl[ii];
    }
    for (size_t ii(0); ii < msg.size(); ++ii) {
      delete msg[ii];
    }
  }
}


TEST (controller, odd_full)
{
  Task * jpos(0);
  Vector gamma_jpos;
  
  vector<Controller*> ctrl;
  vector<ostringstream*> msg;
  vector<Vector> gamma;

  try {
    Model * puma(get_puma());
    Matrix aa;
    Vector gg;
    ASSERT_TRUE (puma->getMassInertia(aa)) << "failed to get mass inertia";
    ASSERT_TRUE (puma->getGravity(gg)) << "failed to get gravity";
    
    jpos = create_sel_jp_task("all", Vector::Ones(puma->getNDOF()));
    Status st;
    st = jpos->init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init jpos task: " << st.errstr;
    st = jpos->update(*puma);
    EXPECT_TRUE (st.ok) << "failed to update jpos task: " << st.errstr;
    gamma_jpos = aa * jpos->getCommand() + gg;
    
    msg.push_back(new ostringstream());
    ctrl.push_back(new SController("Samir", msg.back()));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    msg.push_back(new ostringstream());
    ctrl.push_back(new LController("Luis", msg.back()));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    msg.push_back(new ostringstream());
    ctrl.push_back(new TPController("TaskPosture", msg.back()));
    gamma.push_back(Vector::Zero(puma->getNDOF()));
    
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      
      append_odd_full_tasks(*ctrl[ii], puma->getNDOF());
      st = ctrl[ii]->init(*puma);
      EXPECT_TRUE (st.ok) << "failed to init controller #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
      st = ctrl[ii]->computeCommand(*puma, gamma[ii]);
      EXPECT_TRUE (st.ok) << "failed to compute torques #"
			  << ii << " `" << ctrl[ii]->getName() << "': " << st.errstr;
      
      cout << "==================================================\n"
	   << "messages from controller #" << ii << " `" << ctrl[ii]->getName() << "'\n"
	   << "--------------------------------------------------\n"
	   << msg[ii]->str();
    }
    
    cout << "==================================================\n"
	 << "whole-body torque comparison:\n";
    pretty_print(gamma_jpos, cout, "  reference jpos task", "    ");
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      pretty_print(gamma[ii], cout, "  controller `" + ctrl[ii]->getName() + "'", "    ");
      Vector const delta(gamma_jpos - gamma[ii]);
      pretty_print(delta, cout, "  delta", "    ");
    }
    
  }
  catch (exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
    for (size_t ii(0); ii < ctrl.size(); ++ii) {
      delete ctrl[ii];
    }
    for (size_t ii(0); ii < msg.size(); ++ii) {
      delete msg[ii];
    }
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
