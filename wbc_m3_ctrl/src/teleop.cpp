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
#include <wbc_m3_ctrl/udp_util.h>
#include <wbc_m3_ctrl/qh.h>

#include <ros/ros.h>
#include <jspace/test/sai_util.hpp>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <uta_opspace/ControllerNG.hpp>
#include <wbc_core/opspace_param_callbacks.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <errno.h>
#include <signal.h>

using namespace wbc_m3_ctrl;
using namespace opspace;
using namespace wbc_core_opspace;
using namespace uta_opspace;
using namespace wbcnet;
using namespace boost;
using namespace std;


static char const * opspace_fallback_str = 
  "- tasks:\n"
  "  - type: opspace::CartPosTask\n"
  "    name: eepos\n"
  "    end_effector: right-hand\n"
  "    sigma_threshold: 0.005\n"
  "    control_point: [ 0.0, 0.0, 0.0 ]\n"
  "    kp: [ 60.0 ]\n"
  "    kd: [  0.0 ]\n"
  "    maxvel: [ 0.8 ]\n"
  "  - type: opspace::JPosTask\n"
  "    name: posture\n"
  "    sigma_threshold: 0.0001\n"
  "    kp: [ 0.0 ]\n"
  "    kd: [ 0.0 ]\n"
  "    maxvel: [ 1.0 ]\n"
  "- skills:\n"
  "  - type: opspace::TaskPostureSkill\n"
  "    name: task_posture\n"
  "    default:\n"
  "      eepos: eepos\n"
  "      posture: posture\n";


static bool verbose(false);
static scoped_ptr<jspace::Model> model;
static shared_ptr<Factory> factory;
static shared_ptr<opspace::ReflectionRegistry> registry;
static long long servo_rate;
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
    factory->dump(cerr, "*** parsed tasks and skills", "* ");
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
    shared_ptr<Skill> skill;    
    
    virtual int init(jspace::State const & state) {
      if (skill) {
	warnx("Servo::init(): already initialized");
	return -1;
      }
      if (factory->getSkillTable().empty()) {
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
      
      skill = factory->getSkillTable()[0]; // XXXX to do: allow selection at runtime
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
      if (iteration > 3) {
	warnx("Servo::slowdown(): I'm afraid I can't let you do that");
	return -1;
	
      }
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
  
  int m2s_fd;
  int s2m_fd;
  Parameter * eepos_goal;
  Parameter * eepos_actual;
  controller.reset(new ControllerNG("wbc_m3_ctrl::servo"));
  param_cbs.reset(new ParamCallbacks());
  Servo servo;
  try {
    m2s_fd = wbcnet::create_udp_server(WBC_M3_CTRL_M2S_PORT, AF_UNSPEC);
    s2m_fd = wbcnet::create_udp_client("127.0.0.1", WBC_M3_CTRL_S2M_PORT, AF_UNSPEC);
    
    registry.reset(factory->createRegistry());
    registry->add(controller);
    param_cbs->init(node, registry, 1, 100);
    string errstr;
    eepos_goal = param_cbs->findParam("task", "eepos", "goalpos", errstr);
    if ( ! eepos_goal) {
      throw runtime_error("failed to find eepos goal parameter: " + errstr);
    }
    eepos_actual = param_cbs->findParam("task", "eepos", "actual", errstr);
    if ( ! eepos_actual) {
      throw runtime_error("failed to find eepos actual parameter: " + errstr);
    }
    
    if (verbose) {
      warnx("starting servo with %lld Hz", servo_rate);
    }
    servo.start(servo_rate);
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  warnx("started servo RT thread");
  
  ros::Time t0(ros::Time::now());
  ros::Duration dbg_dt(0.1);

  m2s_data m2s;
  s2m_data s2m;
  struct sockaddr_storage peer_addr;
  socklen_t peer_addr_len;
  Vector const * eepos(eepos_actual->getVector());
  if ( ! eepos) {
    warnx("bug: no vector in eepos_actual");
    ros::shutdown();
  }
  
  Vector master_offset;
  Vector slave_offset(*eepos);
  
  while (ros::ok()) {
    if (verbose) {
      ros::Time t1(ros::Time::now());
      if (t1 - t0 > dbg_dt) {
	t0 = t1;
	cerr << "**************************************************\n";
	jspace::pretty_print(model->getState().position_, cerr, "jpos", "  ");
	jspace::pretty_print(model->getState().velocity_, cerr, "jvel", "  ");
	jspace::pretty_print(model->getState().force_, cerr, "jforce", "  ");
	servo.skill->dbg(cerr, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
	controller->dbg(cerr, "--------------------------------------------------", "");
	cerr << "--------------------------------------------------\n"
	     << "actual eepos\n";
	eepos_actual->dump(cerr, "  ");
	cerr << "eepos goal\n";
	eepos_goal->dump(cerr, "  ");
      }
    }
    
    s2m.eepos_x = eepos->x();
    s2m.eepos_y = eepos->y();
    s2m.eepos_z = eepos->z();
    int const nwritten(udp_client_write(s2m_fd, &s2m, sizeof(s2m)));
    usleep(1000000 / 500);
    
    //too noisy...
    // if (0 > nwritten) {
    //   warn("udp_client_write");
    //   ////      ros::shutdown();
    // }
    
    peer_addr_len = sizeof(struct sockaddr_storage);
    int const nread(udp_server_recvfrom(m2s_fd, &m2s, sizeof(m2s), MSG_DONTWAIT,
					(struct sockaddr *) &peer_addr, &peer_addr_len));
    if (0 > nread) {
      if ((EAGAIN != errno) && (EWOULDBLOCK != errno)) {
	warn("udp_server_recvfrom");
	ros::shutdown();
      }
    }
    else if (sizeof(m2s) == nread) {
      Vector goal(3);
      goal << m2s.eepos_x, m2s.eepos_y, m2s.eepos_z;
      if (0 == master_offset.rows()) {
	master_offset = goal;
      }
      goal -= master_offset;
      goal += slave_offset;
      ////      jspace::pretty_print(goal, cerr, "received goal via UDP", "  ");
      Status const st(eepos_goal->set(goal));
      if ( ! st) {
	warnx("eepos_goal->set() failed: %s", st.errstr.c_str());
	ros::shutdown();
      }
    }
    
    ros::spinOnce();
  }
  
  warnx("shutting down");
  close(m2s_fd);
  servo.shutdown();
}
