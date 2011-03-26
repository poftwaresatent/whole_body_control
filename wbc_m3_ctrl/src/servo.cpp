/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Author: Roland Philippsen
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

#include <wbc_m3_ctrl/rt_util.h>

// one of these just for logging timestamp
#include <rtai_sched.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>

#include <ros/ros.h>
#include <jspace/test/sai_util.hpp>
#include <opspace/Behavior.hpp>
#include <opspace/Factory.hpp>
#include <opspace/ControllerNG.hpp>
#include <wbc_opspace/util.h>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>

using namespace wbc_m3_ctrl;
using namespace opspace;
using namespace wbc_opspace;
using namespace boost;
using namespace std;


static char const * opspace_fallback_str = 
  "- tasks:\n"
  "  - type: opspace::PositionTask\n"
  "    name: eepos\n"
  "    end_effector_id: 6\n"
  "    dt_seconds: 0.002\n"
  "    kp: [ 100.0 ]\n"
  "    kd: [  10.0 ]\n"
  "    maxvel: [ 0.5 ]\n"
  "    maxacc: [ 1.5 ]\n"
  "  - type: opspace::PostureTask\n"
  "    name: posture\n"
  "    dt_seconds: 0.002\n"
  "    kp: [ 100.0 ]\n"
  "    kd: [  10.0 ]\n"
  "    maxvel: [ 3.1416 ]\n"
  "    maxacc: [ 6.2832 ]\n"
  "- behaviors:\n"
  "  - type: opspace::TPBehavior\n"
  "    name: task_posture\n"
  "    default:\n"
  "      eepos: eepos\n"
  "      posture: posture\n";


static bool verbose(false);
static scoped_ptr<jspace::Model> model;
static shared_ptr<Factory> factory;
static long long servo_rate;
static long long actual_servo_rate;
static shared_ptr<ParamCallbacks> param_cbs;
static shared_ptr<ControllerNG> controller;


static void usage(int ecode, std::string msg)
{
  errx(ecode,
       "%s\n"
       "  options:\n"
       "  -h               help (this message)\n"
       "  -v               verbose mode\n"
       "  -r  <filename>   robot specification (SAI XML format)\n"
       "  -f  <frequency>  servo rate (integer number in Hz, default 500Hz)\n"
       "  -s  <filename>   skill specification (YAML file with tasks etc)",
       msg.c_str());
}


static void parse_options(int argc, char ** argv)
{
  string skill_spec("");
  string robot_spec("");
  servo_rate = 500;
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      usage(EXIT_FAILURE, "problem with option `" + string(argv[ii]) + "'");
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(EXIT_SUCCESS, "servo [-h] [-v] [-s skillspec] -r robotspec");
	
      case 'v':
	verbose = true;
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-r requires parameter");
 	}
	robot_spec = argv[ii];
 	break;
	
      case 'f':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-f requires parameter");
 	}
	else {
	  istringstream is(argv[ii]);
	  is >> servo_rate;
	  if ( ! is) {
	    usage(EXIT_FAILURE, "failed to read servo rate from `" + string(argv[ii]) + "'");
	  }
	  if (0 >= servo_rate) {
	    usage(EXIT_FAILURE, "servo rate has to be positive");
	  }
	}
 	break;
	
      case 's':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-s requires parameter");
 	}
	skill_spec = argv[ii];
 	break;
	
      default:
	usage(EXIT_FAILURE, "invalid option `" + string(argv[ii]) + "'");
      }
  }
  
  try {
    if (robot_spec.empty()) {
      usage(EXIT_FAILURE, "no robot specification (see option -r)");
    }
    if (verbose) {
      warnx("reading robot spec from %s", robot_spec.c_str());
    }
    static bool const enable_coriolis_centrifugal(false);
    model.reset(jspace::test::parse_sai_xml_file(robot_spec, enable_coriolis_centrifugal));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE,
	 "exception while parsing robot specification\n"
	 "  filename: %s\n"
	 "  error: %s",
	 robot_spec.c_str(), ee.what());
  }
  
  factory.reset(new Factory());
  
  Status st;
  if (skill_spec.empty()) {
    if (verbose) {
      warnx("using fallback task/posture skill");
    }
    st = factory->parseString(opspace_fallback_str);
  }
  else {
    if (verbose) {
      warnx("reading skills from %s", skill_spec.c_str());
    }
    st = factory->parseFile(skill_spec);
  }
  if ( ! st) {
    errx(EXIT_FAILURE,
	 "failed to parse skills\n"
	 "  specification file: %s\n"
	 "  error description: %s",
	 skill_spec.c_str(), st.errstr.c_str());
  }
  if (verbose) {
    factory->dump(cout, "*** parsed tasks and skills", "* ");
  }
}


static void handle(int signum)
{
  if (ros::ok()) {
    warnx("caught signal, requesting shutdown");
    ros::shutdown();
  }
  else {
    errx(EXIT_SUCCESS, "caught signal (again?), attempting forced exit");
  }
}


namespace {
  
  
  class Servo
    : public RTUtil
  {
  public:
    shared_ptr<Behavior> skill;    
    
    virtual int init(jspace::State const & state) {
      if (skill) {
	warnx("Servo::init(): already initialized");
	return -1;
      }
      if (factory->getBehaviorTable().empty()) {
	warnx("Servo::init(): empty skill table");
	return -2;
      }
      if ( ! model) {
	warnx("Servo::init(): no model");
	return -3;
      }
      
      model->update(state);
    
      jspace::Status status(controller->init(*model));
      if ( ! status) {
	warnx("Servo::init(): controller->init() failed: %s", status.errstr.c_str());
	return -4;
      }
      
      skill = factory->getBehaviorTable()[0]; // XXXX to do: allow selection at runtime
      status = skill->init(*model);
      if ( ! status) {
	warnx("Servo::init(): skill->init() failed: %s", status.errstr.c_str());
	skill.reset();
	return -5;
      }
      
      return 0;
    }
    
    
    virtual int update(jspace::State const & state,
		       jspace::Vector & command)
    {
      if ( ! skill) {
	warnx("Servo::update(): not initialized\n");
	return -1;
      }
      
      model->update(state);
      
      jspace::Status status(controller->computeCommand(*model, *skill, command));
      if ( ! status) {
	warnx("Servo::update(): controller->computeCommand() failed: %s", status.errstr.c_str());
	return -2;
      }
      
      return 0;
    }
    
    
    virtual int cleanup(void)
    {
      skill.reset();
      return 0;
    }
    
    
    virtual int slowdown(long long iteration,
			 long long desired_ns,
			 long long actual_ns)
    {
      actual_servo_rate = 1000000000 / actual_ns;
      return 0;
    }
  };
  
}


int main(int argc, char ** argv)
{
  struct sigaction sa;
  bzero(&sa, sizeof(sa));
  sa.sa_handler = handle;
  if (0 != sigaction(SIGINT, &sa, 0)) {
    err(EXIT_FAILURE, "sigaction");
  }
  
  ros::init(argc, argv, "wbc_m3_ctrl_servo", ros::init_options::NoSigintHandler);
  parse_options(argc, argv);
  ros::NodeHandle node("~");
  
  controller.reset(new ControllerNG("wbc_m3_ctrl::servo"));
  param_cbs.reset(new ParamCallbacks());
  Servo servo;
  try {
    if (verbose) {
      warnx("initializing param callbacks");
    }
    param_cbs->init(node, factory, controller, 1, 100);
    
    if (verbose) {
      warnx("starting servo with %lld Hz", servo_rate);
    }
    actual_servo_rate = servo_rate;
    servo.start(servo_rate);
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "failed to start servo: %s", ee.what());
  }
  
  warnx("started servo RT thread");
  ros::Time dbg_t0(ros::Time::now());
  ros::Time dump_t0(ros::Time::now());
  ros::Duration dbg_dt(0.1);
  ros::Duration dump_dt(0.05);
  
  while (ros::ok()) {
    ros::Time t1(ros::Time::now());
    if (verbose) {
      if (t1 - dbg_t0 > dbg_dt) {
	dbg_t0 = t1;
	servo.skill->dbg(cout, "\n\n**************************************************", "");
	controller->dbg(cout, "--------------------------------------------------", "");
	cout << "--------------------------------------------------\n";
	jspace::pretty_print(model->getState().position_, cout, "jpos", "  ");
	jspace::pretty_print(model->getState().velocity_, cout, "jvel", "  ");
	jspace::pretty_print(model->getState().force_, cout, "jforce", "  ");
	jspace::pretty_print(controller->getCommand(), cout, "gamma", "  ");
	Vector gravity;
	model->getGravity(gravity);
	jspace::pretty_print(gravity, cout, "gravity", "  ");
	cout << "servo rate: " << actual_servo_rate << "\n";
      }
    }
    if (t1 - dump_t0 > dump_dt) {
      dump_t0 = t1;
      controller->qhlog(*servo.skill, rt_get_cpu_time_ns() / 1000);
    }
    ros::spinOnce();
    usleep(10000);		// 100Hz-ish
  }
  
  warnx("shutting down");
  servo.shutdown();
}
