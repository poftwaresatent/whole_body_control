/*
 * Copyright (c) 2010 Stanford University and Willow Garage, Inc.
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
   \file jspace_servo.cpp Pure joint-space controller, with dynamic model and user interaction.
   \author Roland Philippsen
*/

#include <ros/ros.h>
#include <pr2_stanford_wbc/mq_robot_api.h>
#include <pr2_stanford_wbc/ControllerState.h>
#include <pr2_stanford_wbc/SetGoal.h>
#include <pr2_stanford_wbc/SetGains.h>
#include <pr2_stanford_wbc/GetState.h>
#include <pr2_stanford_wbc/GetInfo.h>
#include <pr2_stanford_wbc/SelectController.h>
#include <pr2_stanford_wbc/ControllerInfo.h>
#include <jspace/tao_util.hpp>
#include <stanford_wbc/ros/Model.hpp>
#include <jspace/Model.hpp>
#include <jspace/controller_library.hpp>
#include <jspace/test/sai_brep_parser.hpp>
#include <jspace/test/sai_brep.hpp>
#include <tao/dynamics/taoNode.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <map>

// For initial gains... should maybe move into jspace/ros?
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// To be moved into ../include/pr2_stanford_wbc once it works.
#include "opspace_planar_controller.h"

using namespace pr2_stanford_wbc;
using namespace boost;
using namespace std;

typedef map<string, shared_ptr<jspace::Controller> > controller_lib_t;

static void load_controllers();
static void init_controllers() throw(std::runtime_error);
static jspace::Controller * get_controller(string const & name);
static string const & get_controller_name(jspace::Controller const * ctrl);

static bool set_goal_callback(SetGoal::Request & request, SetGoal::Response & response);
static bool set_gains_callback(SetGains::Request & request, SetGains::Response & response);
static bool get_state_callback(GetState::Request & request, GetState::Response & response);
static bool get_info_callback(GetInfo::Request & request, GetInfo::Response & response);
static bool select_controller_callback(SelectController::Request & request, SelectController::Response & response);

static jspace::State jspace_state;
static scoped_ptr<jspace::Model> jspace_model;
static size_t ndof;
static controller_lib_t controller_lib;
static jspace::Controller * controller;
static bool controllers_initialized(false);


int main(int argc, char*argv[])
{
  ros::init(argc, argv, "jspace_servo", ros::init_options::NoSigintHandler);
  ros::NodeHandle nn("~");
  
  static bool const unlink_mqueue(true);
  MQRobotAPI robot(unlink_mqueue);
  jspace::ros::Model jspace_ros_model("/pr2_stanford_wbc/");
  
  try {
    
    // Quick hack: let people override the robot model from a SAI XML file
    if (argc > 1) {
      
      if ((string(argv[1]) != "-s") || (argc < 3)) {
	throw std::runtime_error("use `-s SAI_XML_FILE' to override robot model");
      }
      ROS_INFO ("creating model via SAI XML file `%s'", argv[2]);
      jspace::test::BRParser brp;
      jspace::test::BranchingRepresentation * brep(brp.parse(argv[2]));
      jspace::tao_tree_info_s * kgm_tree(brep->createTreeInfo());
      jspace::tao_tree_info_s * cc_tree(brep->createTreeInfo());
      
      jspace_model.reset(new jspace::Model());
      if (0 != jspace_model->init(kgm_tree, cc_tree, &cerr)) {
	throw std::runtime_error("jspace_model->init() failed");
      }
      
      ROS_INFO ("gravity compensation thinggie...");
      vector<string> gc_links;
      jspace::ros::Model::
	parseGravityCompensatedLinks(nn, "/pr2_stanford_wbc/gravity_compensated_links", gc_links, 0);
      for (size_t ii(0); ii < gc_links.size(); ++ii) {
	taoDNode const * node(jspace_model->getNodeByName(gc_links[ii]));
	if ( ! node) {
	  ROS_WARN ("no link called `%s', cannot switch its gravity off", gc_links[ii].c_str());
	}
	else {
	  jspace_model->disableGravityCompensation(node->getID(), true);
	}
      }
      
      ROS_INFO ("disposing of temporary jspace::test::BranchingRepresentation");
      delete brep;
      
    }
    else {
      
      ROS_INFO ("creating model via URDF conversion from ROS parameter server");
      static const size_t n_tao_trees(2);
      jspace_ros_model.initFromParam(nn, "/robot_description", n_tao_trees);
      jspace_model.reset(new jspace::Model());
      if (0 != jspace_model->init(jspace_ros_model.tao_trees_[0],
				  jspace_ros_model.tao_trees_[1],
				  &cerr)) {
	throw std::runtime_error("jspace_model->init() failed");
      }
      
      ROS_INFO ("gravity compensation hack...");
      std::vector<std::string>::const_iterator
	gclink(jspace_ros_model.gravity_compensated_links_.begin());
      for (/**/; gclink != jspace_ros_model.gravity_compensated_links_.end(); ++gclink) {
	taoDNode const * node(jspace_model->getNodeByName(*gclink));
	if ( ! node) {
	  throw std::runtime_error("gravity-compensated link " + *gclink
				   + " is not part of the jspace::Model");
	}
	int const id(node->getID());
	jspace_model->disableGravityCompensation(id, true);
	ROS_INFO ("disabled gravity compensation for link %s (ID %d)", gclink->c_str(), id);
      }
    }
    
    ndof = jspace_model->getNDOF();
    jspace_state.init(ndof, ndof, 0);
    
    ROS_INFO ("initializing MQRobotAPI with %zu degrees of freedom", ndof);
    robot.init(true, "pr2_stanford_wbc_s2r", "pr2_stanford_wbc_r2s", ndof, ndof, ndof, ndof);
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  ros::ServiceServer set_goal_server(nn.advertiseService("set_goal", set_goal_callback));
  ros::ServiceServer set_gains_server(nn.advertiseService("set_gains", set_gains_callback));
  ros::ServiceServer get_state_server(nn.advertiseService("get_state", get_state_callback));
  ros::ServiceServer get_info_server(nn.advertiseService("get_info", get_info_callback));
  ros::ServiceServer select_controller_server(nn.advertiseService("select_controller", select_controller_callback));
  
  ros::Publisher state_pub(nn.advertise<ControllerState>("state", 100));
  ControllerState state_msg;
  
  jspace::Vector tau(ndof);
  
  ROS_INFO ("getting initial robot state");
  if ( ! robot.readState(jspace_state)) {
    ROS_ERROR ("robot.readState() failed");
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("entering control loop");
  
  jspace::Vector dbg_g;
  jspace::Matrix dbg_a;
  jspace::Vector dbg_goal;
  jspace::Vector dbg_actual;
  jspace::Vector dbg_velocity;
  jspace::Vector dbg_ext;
  
  jspace::Controller * prev_controller(0);

  while (ros::ok()) {
    
    jspace::Status status;
    
    // We have already read the state, either initially (outside the
    // while loop) or at the previous iteration.
    
    jspace_model->update(jspace_state);
    
    // Lazy init.
    if ( ! controller) {
      try {
	init_controllers();
	controller = get_controller("default");
	if ( ! controller) {
	  throw runtime_error("no default controller");
	}
      }
      catch (std::runtime_error const & ee) {
	ROS_ERROR ("exception during lazy init: %s", ee.what());
	ros::shutdown();
	break;
      }
    }
    
    // Detect controller switch.
    if (prev_controller != controller) {
      status = controller->latch(*jspace_model);
      if ( ! status) {
	ROS_ERROR ("controller->latch() failed: %s", status.errstr.c_str());
	ros::shutdown();
	break;
      }
      prev_controller = controller;
    }
    
    // Do the actual control.
    status = controller->computeCommand(*jspace_model, tau);
    if ( ! status) {
      ROS_ERROR ("controller->computeCommand() failed: %s", status.errstr.c_str());
      ros::shutdown();
      break;
    }
    
    // Send it to the robot.
    status = robot.writeCommand(tau);
    if ( ! status) {
      ROS_ERROR ("robot.writeCommand() failed: %s", status.errstr.c_str());
      ros::shutdown();
      break;
    }

    //////////////////////////////////////////////////
    // Blast out some internal data for visualization and debugging
    
    controller->getGoal(dbg_goal);
    controller->getActual(dbg_actual);
    OpspacePlanarController const * opc(dynamic_cast<OpspacePlanarController const *>(controller));
    if (opc) {
      opc->getDebug(dbg_velocity, state_msg.ext_name, dbg_ext);
    }
    else {
      dbg_velocity = jspace_state.velocity_;
      state_msg.ext_name.clear();
      dbg_ext.resize(0);
    }
    jspace::convert(dbg_goal, state_msg.goal);
    jspace::convert(dbg_actual, state_msg.position);
    jspace::convert(dbg_velocity, state_msg.velocity);
    jspace::convert(tau, state_msg.command);
    jspace::convert(dbg_ext, state_msg.ext_value);
    
    state_pub.publish(state_msg);
    
    ros::spinOnce();
    
    // Read the robot state at the end, so that we have it in the next iteration.
    
    if ( ! robot.readState(jspace_state)) {
      ROS_ERROR ("robot.readState() failed");
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


bool set_goal_callback(SetGoal::Request & request, SetGoal::Response & response)
{
  if ( ! controller) {
    response.ok = false;
    response.errstr = "oops, no controller";
    return true;
  }
  jspace::Status status(controller->setGoal(jspace::Vector::Map(&request.goal[0], request.goal.size())));
  if ( ! status) {
    response.ok = false;
    response.errstr = status.errstr;
    return true;
  }
  response.ok = true;
  response.errstr = "SUCCESS";
  return true;
}


bool set_gains_callback(SetGains::Request & request, SetGains::Response & response)
{
  if ( ! controller) {
    response.ok = false;
    response.errstr = "oops, no controller";
    return true;
  }
  jspace::Status status(controller->setGains(jspace::Vector::Map(&request.kp[0], request.kp.size()),
					     jspace::Vector::Map(&request.kd[0], request.kd.size())));
  if ( ! status) {
    response.ok = false;
    response.errstr = status.errstr;
    return true;
  }
  response.ok = true;
  response.errstr = "SUCCESS";
  return true;
}


bool get_state_callback(GetState::Request & request, GetState::Response & response)
{
  if ( ! controller) {
    response.ok = false;
    response.errstr = "oops, no controller";
    return true;
  }
  try {
    // embedded ControllerInfo
    response.active_controller.controller_name = get_controller_name(controller);
    jspace::controller_info_getter_s const * info_getter(controller->getInfo());
    info_getter->getDOFNames(*jspace_model, response.active_controller.dof_name);
    info_getter->getDOFUnits(*jspace_model, response.active_controller.dof_unit);
    info_getter->getGainNames(*jspace_model, response.active_controller.gain_name);
    jspace::Vector l_lower, l_upper;
    info_getter->getLimits(*jspace_model, l_lower, l_upper);
    response.active_controller.limit_lower.resize(l_lower.size());
    jspace::Vector::Map(&response.active_controller.limit_lower[0], l_lower.size()) = l_lower;
    response.active_controller.limit_upper.resize(l_upper.size());
    jspace::Vector::Map(&response.active_controller.limit_upper[0], l_upper.size()) = l_upper;
    jspace::Vector kp, kd;
    jspace::Status status(controller->getGains(kp, kd));
    if ( ! status) {
      throw runtime_error("controller->getGains(): " + status.errstr);
    }
    response.active_controller.kp.resize(kp.size());
    jspace::Vector::Map(&response.active_controller.kp[0], kp.size()) = kp;
    response.active_controller.kd.resize(kd.size());
    jspace::Vector::Map(&response.active_controller.kd[0], kd.size()) = kd;
    jspace::Vector goal;
    status = controller->getGoal(goal);
    if ( ! status) {
      throw runtime_error("controller->getGoal(): " + status.errstr);
    }
    response.goal.resize(goal.size());
    jspace::Vector::Map(&response.goal[0], goal.size()) = goal;
    jspace::Vector actual;
    status = controller->getActual(actual);
    if ( ! status) {
      throw runtime_error("controller->getActual(): " + status.errstr);
    }
    response.actual.resize(actual.size());
    jspace::Vector::Map(&response.actual[0], actual.size()) = actual;
  }
  catch (std::exception const & ee) {
    response.ok = false;
    response.errstr = ee.what();
    return true;
  }
  response.ok = true;
  response.errstr = "SUCCESS";
  return true;
}


bool get_info_callback(GetInfo::Request & request, GetInfo::Response & response)
{
  try {
    if (controller_lib.empty()) {
      load_controllers();
      init_controllers();
    }
  
    response.controller_info.clear();
    for (controller_lib_t::const_iterator ic(controller_lib.begin()); ic != controller_lib.end(); ++ic) {
      ControllerInfo info;
      info.controller_name = ic->first;
      jspace::controller_info_getter_s const * info_getter(ic->second->getInfo());
      info_getter->getDOFNames(*jspace_model, info.dof_name);
      info_getter->getDOFUnits(*jspace_model, info.dof_unit);
      info_getter->getGainNames(*jspace_model, info.gain_name);
      jspace::Vector foo, bar;
      info_getter->getLimits(*jspace_model, foo, bar);
      jspace::convert(foo, info.limit_lower);
      jspace::convert(bar, info.limit_upper);
      ic->second->getGains(foo, bar);
      jspace::convert(foo, info.kp);
      jspace::convert(bar, info.kd);
      response.controller_info.push_back(info);
    }
    
    response.ok = true;
    response.errstr = "SUCCESS";
    
  }
  catch (std::exception const & ee) {
    response.ok = false;
    response.errstr = string("oops: ") + ee.what();
  }
  
  return true;
}


bool select_controller_callback(SelectController::Request & request, SelectController::Response & response)
{
  try {
    
    if (controller_lib.empty()) {
      load_controllers();
      init_controllers();
    }
    
    jspace::Controller * cc(get_controller(request.controller_name));
    if ( ! cc) {
      throw runtime_error("invalid controller name '" + request.controller_name + "'");
    }
    
    controller = cc;
    response.ok = true;
    response.errstr = "SUCCESS";
    
  }
  catch (std::exception const & ee) {
    response.ok = false;
    response.errstr = ee.what();
  }
  
  return true;
}


jspace::Controller * get_controller(string const & name)
{
  if (controller_lib.empty()) {
    load_controllers();
  }
  
  controller_lib_t::iterator ic;
  if ("default" == name) {
    ic = controller_lib.find("j_goal_g");
  }
  else {
    ic = controller_lib.find(name);
  }
  if (controller_lib.end() == ic) {
    return 0;
  }
  
  return ic->second.get();
}


string const & get_controller_name(jspace::Controller const * ctrl)
{
  if ( ! ctrl) {
    static string const none("(none)");
    return none;
  }
  for (controller_lib_t::const_iterator ic(controller_lib.begin()); ic != controller_lib.end(); ++ic) {
    if (ic->second.get() == ctrl) {
      return ic->first;
    }
  }
  static string const oops("(controller not registered)");
  return oops;
}


static void load_initial_gains(jspace::Vector & initial_kp, jspace::Vector & initial_kd)
{
  std::vector<double> ikp, ikd;
  
  // See stacks/pr2_robot/pr2_controller_configuration/pr2_arm_controllers.yaml for the default values on PR2
  static double const fallback_kp(100);
  static double const fallback_kd(3);
  ikp.assign(ndof, fallback_kp);
  ikd.assign(ndof, fallback_kd);
  
  ros::NodeHandle nn("~");
  XmlRpc::XmlRpcValue gains;
  if ( ! nn.getParam("/pr2_stanford_wbc/gains", gains)) {
    ROS_WARN ("no /pr2_stanford_wbc/gains -- using fallback kp = %f and kd = %f",
	      fallback_kp, fallback_kd);
    return;
  }
  
  try {
    
    XmlRpc::XmlRpcValue::iterator igain(gains.begin());
    XmlRpc::XmlRpcValue::iterator igain_end(gains.end());
    for (/**/; igain != igain_end; ++igain) {
      std::string const & dof_name(igain->first);
      ROS_INFO ("  dof_name = %s", dof_name.c_str());
      
      taoDNode const * node;
      node = jspace_model->getNodeByName(dof_name);
      if ( ! node) {
	node = jspace_model->getNodeByJointName(dof_name);
      }
      if ( ! node) {
	ROS_WARN ("  no match for %s in node or joint names", dof_name.c_str());
	continue;
      }
      int const id(node->getID());
      if ((0 > id) || (ndof <= id)) {
	ROS_WARN ("  invalid ID %d for node %s", id, dof_name.c_str());
	continue;
      }
      
      XmlRpc::XmlRpcValue /*const*/ & entries(igain->second);
      XmlRpc::XmlRpcValue::iterator ientry(entries.begin());
      XmlRpc::XmlRpcValue::iterator ientry_end(entries.end());
      for (/**/; ientry != ientry_end; ++ientry) {
	std::string const gain_name(ientry->first);
	double gain_value;
	if (ientry->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
	  gain_value = static_cast<int const &>(ientry->second);
	}
	else {
	  gain_value = static_cast<double const &>(ientry->second);
	}
	
	if ("kp" == gain_name) {
	  ikp[id] = gain_value;
	}
	else if ("kd" == gain_name) {
	  ikd[id] = gain_value;
	}
	else {
	  ROS_WARN ("    invalid gain name %s", gain_name.c_str());
	}

	ROS_INFO ("    gain: %s[%d] = %f", gain_name.c_str(), id, gain_value);
      }
    }
    
  }
  catch (XmlRpc::XmlRpcException const & ee) {
    ROS_WARN ("XmlRpcException while reading gains: %s", ee.getMessage().c_str());
  }
  
  jspace::convert(ikp, initial_kp);
  jspace::convert(ikd, initial_kd);
}


void load_controllers() {
  if ( ! controller_lib.empty()) {
    return;
  }
  
  jspace::Vector initial_kp, initial_kd;
  load_initial_gains(initial_kp, initial_kd);
  
  shared_ptr<jspace::Controller> ctrl;
    
  ctrl.reset(new jspace::FloatController());
  controller_lib.insert(make_pair("float", ctrl));
  
  ctrl.reset(new jspace::JointGoalController(jspace::COMP_GRAVITY,
					     initial_kp, initial_kd));
  controller_lib.insert(make_pair("j_goal_g", ctrl));
    
  ctrl.reset(new jspace::JointGoalController(jspace::COMP_GRAVITY |
					     jspace::COMP_MASS_INERTIA,
					     initial_kp, initial_kd));
  controller_lib.insert(make_pair("j_goal_Ag", ctrl));
    
  ctrl.reset(new jspace::JointGoalController(jspace::COMP_GRAVITY |
					     jspace::COMP_CORIOLIS |
					     jspace::COMP_MASS_INERTIA,
					     initial_kp, initial_kd));
  controller_lib.insert(make_pair("j_goal_Abg", ctrl));
  
  ctrl.reset(OpspacePlanarController::create("/opspace_planar_controller", initial_kp, initial_kd));
  if ( ! ctrl) {
    ROS_WARN ("failed to create planar controller from `/opspace_planar_controller', falling back on defaults");
    double const op_kp(25);
    double const op_kd(10);
    double const op_vmax(0.1);
    std::string const q1_name("l_shoulder_lift_link");
    bool const q1_inverted(false);
    double const l1_length(0.4);
    std::string q2_name("l_elbow_flex_link");
    bool const q2_inverted(false);
    double const l2_length(0.321);
    ctrl.reset(new OpspacePlanarController(q1_name, q1_inverted, l1_length,
					   q2_name, q1_inverted, l2_length,
					   op_kp, op_kd, op_vmax,
					   initial_kp, initial_kd));
  }
  controller_lib.insert(make_pair("op_planar", ctrl));
}


void init_controllers() throw(std::runtime_error)
{
  if (controllers_initialized) {
    return;
  }
  
  if ( ! jspace_model) {
    throw runtime_error("someone called init_controllers() but the model is not ready");
  }
  
  load_controllers();
  std::list<std::string> unload;
  for (controller_lib_t::iterator ic(controller_lib.begin()); ic != controller_lib.end(); ++ic) {
    jspace::Status const status(ic->second->init(*jspace_model));
    if ( ! status) {
      ROS_WARN ("Removing controller `%s' because it failed to initialize: %s",
		ic->first.c_str(), status.errstr.c_str());
      ic->second.reset();
      unload.push_back(ic->first);
    }
  }
  
  for (std::list<std::string>::const_iterator iu(unload.begin());
       iu != unload.end(); ++iu) {
    controller_lib.erase(*iu);
  }
  if (controller_lib.empty()) {
    throw runtime_error("zarroh controhlers initialisationized");
  }
  
  controllers_initialized = true;
}
