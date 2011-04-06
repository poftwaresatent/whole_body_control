/*
 * Copyright (c) 2011 Stanford University and Willow Garage, Inc.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file opspace_servo.cpp Operational-space controller, uses wbc_opspace package.
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <wbc_pr2_ctrl/mq_robot_api.h>
#include <wbc_pr2_ctrl/PumpGetInfo.h>
#include <wbc_urdf/Model.hpp>
#include <jspace/Model.hpp>
#include <jspace/test/sai_util.hpp>
#include <tao/dynamics/taoNode.h>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <opspace/controller_library.hpp>
#include <wbc_opspace/util.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <err.h>

using namespace wbc_pr2_ctrl;
using namespace wbc_msgs;
using namespace opspace;
using namespace boost;
using namespace std;


static char const * opspace_fallback_str = 
  "- tasks:\n"
  "  - type: opspace::PositionTask\n"
  "    name: eepos_instance\n"
  "    end_effector_id: 6\n"
  "    dt_seconds: 0.002\n"
  "    kp: [ 100.0 ]\n"
  "    kd: [  20.0 ]\n"
  "    maxvel: [ 0.5 ]\n"
  "    maxacc: [ 1.5 ]\n"
  "  - type: opspace::PostureTask\n"
  "    name: posture_instance\n"
  "    dt_seconds: 0.002\n"
  "    kp: [ 100.0 ]\n"
  "    kd: [  20.0 ]\n"
  "    maxvel: [ 3.1416 ]\n"
  "    maxacc: [ 6.2832 ]\n"
  "- skills:\n"
  "  - type: opspace::TPSkill\n"
  "    name: tpb\n"
  "    slots:\n"
  "      eepos: eepos_instance\n"
  "      posture: posture_instance\n";


static jspace::State pump_state;
static jspace::State servo_state;
static vector<int> pump_to_servo_id;

static MQRobotAPI robot(true);

static scoped_ptr<jspace::Model> jspace_model;
static size_t pump_ndof;
static size_t servo_ndof;
static shared_ptr<opspace::Factory> factory;
static shared_ptr<opspace::ReflectionRegistry> registry;
static shared_ptr<opspace::Controller> controller;
static wbc_opspace::ParamCallbacks param_callbacks;

// I think this needs to stick around because otherwise it deletes the TAO nodes
static scoped_ptr<jspace::ros::Model> jspace_ros_model;

static bool update_model_from_pump();


int main(int argc, char*argv[])
{
  //////////////////////////////////////////////////
  // init
  
  ros::init(argc, argv, "opspace_servo", ros::init_options::NoSigintHandler);
  ros::NodeHandle nn("~");
  factory.reset(new opspace::Factory());
  
  //////////////////////////////////////////////////
  // parse options
  
  std::string opspace_filename("");
  std::string tao_filename("");
  bool verbose(false);
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      errx(EXIT_FAILURE, "problem with option `%s'", argv[ii]);
    }
    else
      switch (argv[ii][1]) {
	
      case 'b':
 	++ii;
 	if (ii >= argc) {
	  errx(EXIT_FAILURE, "-b requires parameter");
 	}
	opspace_filename = argv[ii];
 	break;
	
      case 't':
 	++ii;
 	if (ii >= argc) {
	  errx(EXIT_FAILURE, "-t requires parameter");
 	}
	tao_filename = argv[ii];
 	break;
	
      case 'v':
	verbose = true;
 	break;
	
      default:
	errx(EXIT_FAILURE, "invalid option `%s'", argv[ii]);
      }
  }
  
  //////////////////////////////////////////////////
  // initialize
  
  PumpGetInfo::Response pump_info;
  ROS_INFO ("retrieving info from (hopefully already running) pump plugin");
  {
    static char const * srvname("/wbc_pr2_ctrl_pump_plugin/get_info");
    ros::ServiceClient client(nn.serviceClient<PumpGetInfo>(srvname));
    PumpGetInfo srv;
    if ( ! client.call(srv)) {
      ROS_ERROR ("failed to call service `%s'", srvname);
      errx(EXIT_FAILURE, "failed to call service `%s'", srvname);
    }
    if (srv.response.joint_name.empty()) {
      ROS_ERROR ("no joint names in reply from pump");
      errx(EXIT_FAILURE, "no joint names in reply from pump");
    }
    for (size_t ii(0); ii < srv.response.joint_name.size(); ++ii) {
      ROS_INFO ("  %zu: %s", ii, srv.response.joint_name[ii].c_str());
    }
    pump_info = srv.response;
  }
  
  pump_ndof = pump_info.joint_name.size();
  ROS_INFO ("initializing MQRobotAPI: %zu DOF  wr: %s  rd: %s",
	    pump_ndof,
	    pump_info.mq_name_servo_to_pump.c_str(), 
	    pump_info.mq_name_pump_to_servo.c_str());
  robot.init(true,
	     pump_info.mq_name_servo_to_pump,
	     pump_info.mq_name_pump_to_servo,
	     pump_ndof, pump_ndof, pump_ndof, pump_ndof);
  
  try {
    
    if (tao_filename.empty()) {
      ROS_INFO ("creating model via URDF conversion from ROS parameter server");
      jspace_ros_model.reset(new jspace::ros::Model("/wbc_pr2_ctrl/"));
      static const size_t n_tao_trees(2);
      jspace_ros_model->initFromParam(nn, "/robot_description", n_tao_trees);
      jspace_model.reset(new jspace::Model());
      if (0 != jspace_model->init(jspace_ros_model->tao_trees_[0],
				  jspace_ros_model->tao_trees_[1],
				  &cerr)) {
	throw std::runtime_error("jspace_model->init() failed");
      }
    }
    
    else {
      ROS_INFO ("creating model from SAI XML file %s", tao_filename.c_str());
      static bool const enable_coriolis_centrifugal(true);
      jspace_model.reset(jspace::test::parse_sai_xml_file(tao_filename, enable_coriolis_centrifugal));
    }
    
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  servo_ndof = jspace_model->getNDOF();
  
  XmlRpc::XmlRpcValue gc_links_value;
  std::string const gc_links_param_name("/wbc_pr2_ctrl/gravity_compensated_links");
  if ( ! nn.getParam(gc_links_param_name, gc_links_value)) {
    ROS_INFO ("no parameter called `%s': skipping gravity compensation",
	      gc_links_param_name.c_str());
  }
  else {
    ROS_INFO ("switching off gravity compensation...");
    try {
      for (int ii(0); ii < gc_links_value.size(); ++ii) {
	std::string const linkname(static_cast<std::string const &>(gc_links_value[ii]));
	taoDNode const * node(jspace_model->getNodeByName(linkname));
	if ( ! node) {
	  ROS_WARN ("...gravity-compensated link `%s' is not part of the jspace::Model",
		    linkname.c_str());
	  continue;
	}
	int const id(node->getID());
	jspace_model->disableGravityCompensation(id, true);
	ROS_INFO ("...disabled gravity for link `%s' (ID %d)", linkname.c_str(), id);
      }
    }
    catch (XmlRpc::XmlRpcException const & ee) {
      ROS_ERROR ("XmlRpcException while reading gravity compensated links: %s",
		 ee.getMessage().c_str());
      errx(EXIT_FAILURE, "XmlRpcException while reading gravity compensated links: %s",
	   ee.getMessage().c_str());
    }
  }
  
  ROS_INFO ("initializing joint states with %zu DOF (pump) and %zu DOF (servo) and mapping between them",
	    pump_ndof, servo_ndof);
  pump_state.init(pump_ndof, pump_ndof, pump_ndof);
  servo_state.init(servo_ndof, servo_ndof, servo_ndof);
  pump_to_servo_id.clear();
  for (size_t ii(0); ii < pump_ndof; ++ii) {
    std::string const & jointname(pump_info.joint_name[ii]);
    taoDNode const * node(jspace_model->getNodeByJointName(jointname));
    if ( ! node) {
      ROS_ERROR ("pumped joint `%s' is not in jspace::Model", jointname.c_str());
      errx (EXIT_FAILURE, "pumped joint `%s' is not in jspace::Model", jointname.c_str());
    }
    int const id(node->getID());
    if (0 > id) {
      ROS_ERROR ("pumped joint `%s' is root node of jspace::Model", jointname.c_str());
      errx (EXIT_FAILURE, "pumped joint `%s' is root node of jspace::Model", jointname.c_str());
    }
    ROS_INFO ("  pump ID: %zu  servo ID: %d  joint %s", ii, id, jointname.c_str());
    pump_to_servo_id.push_back(id);
  }
  
  ROS_INFO ("parsing opspace tasks and skills");
  shared_ptr<Skill> skill; // for now, just use the first one we encounter
  try {
    
    Status st;
    if (opspace_filename.empty()) {
      ROS_WARN ("no opspace_filename -- using fallback task/posture skill");
      st = factory->parseString(opspace_fallback_str);
    }
    else {
      ROS_INFO ("opspace filename is %s", opspace_filename.c_str());
      st = factory->parseFile(opspace_filename);
    }
    if ( ! st) {
      throw runtime_error("failed to parse opspace tasks and skills: " + st.errstr);
    }
    factory->dump(cout, "*** dump of opspace task/skill factory", "* ");
    if (factory->getTaskTable().empty()) {
      throw runtime_error("empty opspace task table");
    }
    if (factory->getSkillTable().empty()) {
      throw runtime_error("empty opspace skill table");
    }
    skill = factory->getSkillTable()[0]; // XXXX to do: allow selection at runtime
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("initializing state, model, tasks, skills, ...");
  if ( ! update_model_from_pump()) {
    ROS_ERROR ("update_model_from_pump() failed");
    exit(EXIT_FAILURE);
  }
  controller.reset(new ControllerNG(""));
  ////  controller.reset(new ClassicTaskPostureController(""));
  jspace::Status status;
  status = controller->init(*jspace_model);
  if ( ! status) {
    ROS_ERROR ("controller->init() failed: %s", status.errstr.c_str());
    exit(EXIT_FAILURE);
  }
  status = skill->init(*jspace_model);
  if ( ! status) {
    ROS_ERROR ("skill->init() failed: %s", status.errstr.c_str());
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("starting services");
  registry.reset(factory->createRegistry());
  registry->add(controller);	// one day this will come from factory, too
  try {
    param_callbacks.init(nn, registry, 1, 100);
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("entering control loop");
  jspace::Vector servo_tau(jspace::Vector::Zero(servo_ndof));
  jspace::Vector pump_tau(jspace::Vector::Zero(pump_ndof));
  ros::WallTime t0(ros::WallTime::now());
  ros::WallDuration dbg_dt(0.2);
  while (ros::ok()) {
    
    // Compute torque command.
    status = controller->computeCommand(*jspace_model, *skill, servo_tau);
    if ( ! status) {
      ROS_ERROR ("controller->computeCommand() failed: %s", status.errstr.c_str());
      ros::shutdown();
      break;
    }
    
    // Send it to the robot.
    for (size_t ii(0); ii < pump_ndof; ++ii) {
      pump_tau[ii] = servo_tau[pump_to_servo_id[ii]];
    }
    status = robot.writeCommand(pump_tau);
    if ( ! status) {
      ROS_ERROR ("robot.writeCommand() failed: %s", status.errstr.c_str());
      ros::shutdown();
      break;
    }
    
    if (verbose) {
      ros::WallTime t1(ros::WallTime::now());
      if (t1 - t0 > dbg_dt) {
	t0 = t1;
	skill->dbg(cout, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
	controller->dbg(cout, "--------------------------------------------------", "");
      }
    }
    
    // Wait for "tick" (haha, this is not RT anyway...)
    ros::spinOnce();
    
    // Read the robot state and update the model
    if ( ! update_model_from_pump()) {
      ROS_ERROR ("update_model_from_pump() failed");
      ros::shutdown();
      break;
    }
    
  }

  for (size_t ii(10); ros::ok() && (ii > 0); --ii) {
    if (10 == ii) {
      cerr << "waiting 10 seconds for node shutdown...";
    }
    cerr << ii << "..." << flush;
    usleep(1000000);
  }
  if (ros::ok()) {
    cerr << "giving up\n";
  }
  else {
    cerr << "done\n";
  }
}


bool update_model_from_pump()
{  
  if ( ! robot.readState(pump_state)) {
    ROS_ERROR ("update_model_from_pump(): robot.readState() failed");
    return false;
  }
  if ((pump_to_servo_id.size() != pump_state.position_.size())
      || (pump_to_servo_id.size() != pump_state.velocity_.size())
      || (pump_to_servo_id.size() != pump_state.force_.size())) {
    ROS_ERROR ("update_model_from_pump(): pump_to_servo_id size %zu mismatch: pos %zu  vel %zu  force %zu",
	       pump_to_servo_id.size(),
	       pump_state.position_.size(),
	       pump_state.velocity_.size(),
	       pump_state.force_.size());
    return false;
  }
  for (size_t ii(0); ii < pump_to_servo_id.size(); ++ii) {
    int const servo_idx(pump_to_servo_id[ii]);
    if (0 > servo_idx) {
      ROS_ERROR ("update_model_from_pump(): pump idx %zu  servo idx %d < 0", ii, servo_idx);
      return false;
    }
    if (servo_ndof <= servo_idx) {
      ROS_ERROR ("update_model_from_pump(): pump idx %zu  servo idx %d >= NDOF of %zu", ii, servo_idx, servo_ndof);
      return false;
    }
    servo_state.position_[servo_idx] = pump_state.position_[ii];
    servo_state.velocity_[servo_idx] = pump_state.velocity_[ii];
    servo_state.force_[servo_idx] = pump_state.force_[ii];
  }
  jspace_model->update(servo_state);
  return true;
}
