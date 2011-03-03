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
#include <opspace/other_controllers.hpp>
#include <jspace/test/model_library.hpp>
#include <err.h>

using jspace::Model;
using jspace::State;
using jspace::pretty_print;
using namespace opspace;
using boost::shared_ptr;
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
  
  Parameter * selection(odd.lookupParameter("selection", PARAMETER_TYPE_VECTOR));
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


static shared_ptr<Task> create_sel_jp_task(string const & name, Vector const & selection)
  throw(runtime_error)
{
  SelectedJointPostureTask * task(new SelectedJointPostureTask(name));
  Parameter * sel_p(task->lookupParameter("selection", PARAMETER_TYPE_VECTOR));
  if ( ! sel_p) {
    delete task;
    throw runtime_error("failed to retrieve selection parameter");
  }
  Status const st(sel_p->set(selection));
  if ( ! st) {
    delete task;
    throw runtime_error("failed to set selection: " + st.errstr);
  }
  return shared_ptr<Task>(task);
}


static void append_odd_even_tasks(Controller & ctrl, size_t ndof)
  throw(runtime_error)
{
  vector<shared_ptr<Task> > task;
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
    if ( ! ctrl.appendTask(task[ii])) {
      throw runtime_error("failed to add task `" + task[ii]->getName() + "'");
    }
  }
}


static void append_odd_full_tasks(Controller & ctrl, size_t ndof)
  throw(runtime_error)
{
  vector<shared_ptr<Task> > task;
  Vector sel_odd(Vector::Zero(ndof));
  Vector sel_full(Vector::Ones(ndof));
  for (size_t ii(1); ii < ndof; ii += 2) {
    sel_odd[ii] = 1.0;
  }
  task.push_back(create_sel_jp_task("odd", sel_odd));
  task.push_back(create_sel_jp_task("full", sel_full));
  for (size_t ii(0); ii < task.size(); ++ii) {
    if ( ! ctrl.appendTask(task[ii])) {
      throw runtime_error("failed to add task `" + task[ii]->getName() + "'");
    }
  }
}


TEST (controller, odd_even)
{
  shared_ptr<Task> jpos;
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
  shared_ptr<Task> jpos;
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




TEST (task, jlimit)
{
  shared_ptr<JointLimitTask> jlimit(new JointLimitTask("jlimit"));
  
  try {
    Model * puma(get_puma());
    size_t const ndof(puma->getNDOF());
    
    Parameter * param(jlimit->lookupParameter("dt_seconds", PARAMETER_TYPE_REAL));
    ASSERT_NE ((void*)0, param) << "failed to get dt_seconds param";
    Status st(param->set(0.1));
    ASSERT_TRUE (st.ok) << "failed to set dt_seconds: " << st.errstr;

    param = jlimit->lookupParameter("upper_stop_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get upper_stop_deg param";
    Vector foo(30.0 * Vector::Ones(ndof));
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set upper_stop_deg: " << st.errstr;

    param = jlimit->lookupParameter("upper_trigger_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get upper_trigger_deg param";
    foo = 20.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set upper_trigger_deg: " << st.errstr;

    param = jlimit->lookupParameter("lower_stop_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get lower_stop_deg param";
    foo = -30.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set lower_stop_deg: " << st.errstr;

    param = jlimit->lookupParameter("lower_trigger_deg", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get lower_trigger_deg param";
    foo = -20.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set lower_trigger_deg: " << st.errstr;

    param = jlimit->lookupParameter("kp", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get kp param";
    foo = 100.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set kp: " << st.errstr;

    param = jlimit->lookupParameter("kd", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get kd param";
    foo = 20.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set kd: " << st.errstr;

    param = jlimit->lookupParameter("maxvel", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get maxvel param";
    foo = 10.0 * M_PI / 180.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set maxvel: " << st.errstr;

    param = jlimit->lookupParameter("maxacc", PARAMETER_TYPE_VECTOR);
    ASSERT_NE ((void*)0, param) << "failed to get maxacc param";
    foo = 25.0 * M_PI / 180.0 * Vector::Ones(ndof);
    st = param->set(foo);
    ASSERT_TRUE (st.ok) << "failed to set maxacc: " << st.errstr;
    
    LController ctrl("ctrl", &cerr);
    ctrl.appendTask(jlimit);
    
    State state(ndof, ndof, 0);
    state.position_ = Vector::Zero(ndof);
    state.velocity_ = Vector::Zero(ndof);
    puma->update(state);
    st = ctrl.init(*puma);
    EXPECT_TRUE (st.ok) << "failed to init: " << st.errstr;
    Vector gamma;
    st = ctrl.computeCommand(*puma, gamma);
    EXPECT_TRUE (st.ok) << "failed to computeCommand: " << st.errstr;
    
    for (size_t ii(0); ii < ndof; ++ii) {
      if (0 == ii % 2) {
	state.position_[ii] =  4.0 * M_PI;
      }
      else {
	state.position_[ii] = -4.0 * M_PI;
      }
      puma->update(state);
      st = ctrl.computeCommand(*puma, gamma);
      EXPECT_TRUE (st.ok) << "failed to computeCommand: " << st.errstr;
    }
    
  }
  catch (exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}
