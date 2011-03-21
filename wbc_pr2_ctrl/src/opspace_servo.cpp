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
#include <wbc_urdf/Model.hpp>
#include <jspace/Model.hpp>
#include <tao/dynamics/taoNode.h>
#include <opspace/Behavior.hpp>
#include <opspace/Factory.hpp>
#include <opspace/ControllerNG.hpp>
#include <wbc_msgs/SetParameter.h>
#include <wbc_msgs/GetParameter.h>
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
  "- behaviors:\n"
  "  - type: opspace::TPBehavior\n"
  "    name: tpb\n"
  "    default:\n"
  "      eepos: eepos_instance\n"
  "      posture: posture_instance\n";


static jspace::State jspace_state;
static scoped_ptr<jspace::ros::Model> jspace_ros_model;
static scoped_ptr<jspace::Model> jspace_model;
static size_t ndof;
static opspace::Factory factory;
static shared_ptr<ControllerNG> controller;


static bool set_parameter_callback(SetParameter::Request & request,
				   SetParameter::Response & response);

static bool get_parameter_callback(GetParameter::Request & request,
				   GetParameter::Response & response);


int main(int argc, char*argv[])
{
  //////////////////////////////////////////////////
  // init
  
  ros::init(argc, argv, "opspace_servo", ros::init_options::NoSigintHandler);
  ros::NodeHandle nn("~");
  
  //////////////////////////////////////////////////
  // parse options
  
  std::string opspace_filename("");
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
	
      case 'v':
	verbose = true;
 	break;
	
      default:
	errx(EXIT_FAILURE, "invalid option `%s'", argv[ii]);
      }
  }
  
  static bool const unlink_mqueue(true);
  MQRobotAPI robot(unlink_mqueue);
  
  ROS_INFO ("creating model via URDF conversion from ROS parameter server");
  try {
    
    jspace_ros_model.reset(new jspace::ros::Model("/wbc_pr2_ctrl/"));
    static const size_t n_tao_trees(2);
    jspace_ros_model->initFromParam(nn, "/robot_description", n_tao_trees);
    jspace_model.reset(new jspace::Model());
    if (0 != jspace_model->init(jspace_ros_model->tao_trees_[0],
				jspace_ros_model->tao_trees_[1],
				&cerr)) {
      throw std::runtime_error("jspace_model->init() failed");
    }
    
    ROS_INFO ("gravity compensation hack...");
    std::vector<std::string>::const_iterator
      gclink(jspace_ros_model->gravity_compensated_links_.begin());
    for (/**/; gclink != jspace_ros_model->gravity_compensated_links_.end(); ++gclink) {
      taoDNode const * node(jspace_model->getNodeByName(*gclink));
      if ( ! node) {
	throw std::runtime_error("gravity-compensated link " + *gclink
				 + " is not part of the jspace::Model");
      }
      int const id(node->getID());
      jspace_model->disableGravityCompensation(id, true);
      ROS_INFO ("disabled gravity compensation for link %s (ID %d)", gclink->c_str(), id);
    }
    
    ndof = jspace_model->getNDOF();
    jspace_state.init(ndof, ndof, 0);
    
    ROS_INFO ("initializing MQRobotAPI with %zu degrees of freedom", ndof);
    robot.init(true, "wbc_pr2_ctrl_s2r", "wbc_pr2_ctrl_r2s", ndof, ndof, ndof, ndof);
  }
  
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }

  ROS_INFO ("parsing opspace tasks and behaviors");
  shared_ptr<Behavior> behavior; // for now, just use the first one we encounter
  try {
    
    Status st;
    if (opspace_filename.empty()) {
      ROS_WARN ("no opspace_filename -- using fallback task/posture behavior");
      st = factory.parseString(opspace_fallback_str);
    }
    else {
      ROS_INFO ("opspace filename is %s", opspace_filename.c_str());
      st = factory.parseFile(opspace_filename);
    }
    if ( ! st) {
      throw runtime_error("failed to parse opspace tasks and behaviors: " + st.errstr);
    }
    factory.dump(cout, "*** dump of opspace task/behavior factory", "* ");
    if (factory.getTaskTable().empty()) {
      throw runtime_error("empty opspace task table");
    }
    if (factory.getBehaviorTable().empty()) {
      throw runtime_error("empty opspace behavior table");
    }
    behavior = factory.getBehaviorTable()[0]; // XXXX to do: allow selection at runtime
  }
  catch (std::exception const & ee) {
    ROS_ERROR ("EXCEPTION %s", ee.what());
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("initializing state, model, tasks, behaviors, ...");
  if ( ! robot.readState(jspace_state)) {
    ROS_ERROR ("robot.readState() failed");
    exit(EXIT_FAILURE);
  }
  jspace_model->update(jspace_state);
  controller.reset(new ControllerNG("opspace_servo"));
  jspace::Status status;
  status = controller->init(*jspace_model);
  if ( ! status) {
    ROS_ERROR ("controller->init() failed: %s", status.errstr.c_str());
    exit(EXIT_FAILURE);
  }
  status = behavior->init(*jspace_model);
  if ( ! status) {
    ROS_ERROR ("behavior->init() failed: %s", status.errstr.c_str());
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO ("starting services");
  ros::ServiceServer set_parameter_server(nn.advertiseService("set_parameter", set_parameter_callback));
  ros::ServiceServer get_parameter_server(nn.advertiseService("get_parameter", get_parameter_callback));
  
  ROS_INFO ("entering control loop");
  jspace::Vector tau(ndof);
  while (ros::ok()) {
    
    // Compute torque command.
    status = controller->computeCommand(*jspace_model, *behavior, tau);
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
    
    if (verbose) {
      behavior->dbg(cout, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", "");
      controller->dbg(cout, "--------------------------------------------------", "");
    }
    
    // Wait for "tick" (haha, this is not RT anyway...)
    ros::spinOnce();
    
    // Read the robot state and update the model
    if ( ! robot.readState(jspace_state)) {
      ROS_ERROR ("robot.readState() failed");
      ros::shutdown();
      break;
    }
    jspace_model->update(jspace_state);
    
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


static Parameter * find_param(std::string const & com_type,
			      std::string const & com_name,
			      std::string const & param_name,
			      std::string & errstr)
{
  if ("task" == com_type) {
    shared_ptr<Task> task(factory.findTask(com_name));
    if ( ! task) {
      errstr = "no such task";
      return 0;
    }
    errstr = "trying task parameters...";
    return task->lookupParameter(param_name);
  }
  else if ("skill" == com_type) {
    shared_ptr<Behavior> skill(factory.findBehavior(com_name));
    if ( ! skill) {
      errstr = "no such skill";
      return 0;
    }
    errstr = "trying skill parameters...";
    return skill->lookupParameter(param_name);
  }
  else if ("servo" == com_type) {
    // ignore com_name for now...
    errstr = "trying servo parameters...";
    return controller->lookupParameter(param_name);
  }
  errstr = "invalid com_type (use task, skill, or servo)";
  return 0;
}


bool set_parameter_callback(SetParameter::Request & request,
			    SetParameter::Response & response)
{
  Status status;
  Parameter * param(find_param(request.com_type, request.com_name, request.param.name, status.errstr));
  if ( ! param) {
    status.ok = false;
    // status.errstr set by find_param...
    return true;
  }
  
  switch (request.param.type) {
    
  case OpspaceParameter::PARAMETER_TYPE_STRING:
    status = param->set(request.param.strval);
    break;
    
  case OpspaceParameter::PARAMETER_TYPE_INTEGER:
    status = param->set((int) request.param.intval); // grr, one day the 32 vs 64 bit thing will bite us
    break;
    
  case OpspaceParameter::PARAMETER_TYPE_REAL:
    if (1 != request.param.realval.size()) {
      status.ok = false;
      status.errstr = "expected exactly one realval";
    }
    else {
      status = param->set(request.param.realval[0]);
    }
    break;
    
  case OpspaceParameter::PARAMETER_TYPE_VECTOR:
    {
      // I tried to find a more or less elegant and non-intrusive way
      // to do without the tmp, and failed. Eigen is simply not made
      // to mix well with runtime dynamic typing. For instance,
      // Eigen::Map<> can be made to look "just like" Eigen::Matrix<>
      // to the compiler in expressions, but trying to pass a const
      // ref to a map in some place which expects a const ref to a
      // vector simply does not work.
      Vector tmp(jspace::Vector::Map(&request.param.realval[0],
				     request.param.realval.size()));
      status = param->set(tmp);
    }
    break;
    
  case OpspaceParameter::PARAMETER_TYPE_MATRIX:
    if ((0 > request.param.nrows) || (0 > request.param.ncols)) {
      status.ok = false;
      status.errstr = "invalid matrix dimensions";
    }
    else if (request.param.realval.size() != request.param.nrows * request.param.ncols) {
      status.ok = false;
      status.errstr = "matrix dimension mismatch";
    }
    else {
      // See comments about Eigen::Map<> above.
      Matrix tmp(jspace::Vector::Map(&request.param.realval[0],
				     request.param.nrows,
				     request.param.ncols));
      status = param->set(tmp);
    }
    break;
    
  default:
    status.ok = false;
    status.errstr = "unsupported or invalid type";
  }
  
  response.ok = status.ok;
  response.errstr = status.errstr;
  return true;
}


bool get_parameter_callback(GetParameter::Request & request,
			    GetParameter::Response & response)
{
  Status status;
  Parameter const * param(find_param(request.com_type, request.com_name, request.param_name, status.errstr));
  if ( ! param) {
    status.ok = false;
    // status.errstr set by find_param...
    return true;
  }
  
  response.param.name = request.param_name;
  response.param.type = OpspaceParameter::PARAMETER_TYPE_VOID;
  
  switch (param->type_) {
    
  case PARAMETER_TYPE_STRING:
    response.param.type = OpspaceParameter::PARAMETER_TYPE_STRING;
    response.param.strval = *param->getString();
    break;
    
  case PARAMETER_TYPE_INTEGER:
    response.param.type = OpspaceParameter::PARAMETER_TYPE_INTEGER;
    response.param.intval = *param->getInteger();
    break;
    
  case PARAMETER_TYPE_REAL:
    response.param.type = OpspaceParameter::PARAMETER_TYPE_REAL;
    response.param.realval.resize(1);
    response.param.realval[0] = *param->getReal();
    break;
    
  case PARAMETER_TYPE_VECTOR:
    response.param.type = OpspaceParameter::PARAMETER_TYPE_VECTOR;
    {
      Vector const * vv(param->getVector());
      if ( ! vv) {
	response.ok = false;
	response.errstr = "oops, looks like a bug: no vector in vector parameter";
	return true;
      }
      response.param.realval.resize(vv->rows());
      Vector::Map(&response.param.realval[0], vv->rows()) = *vv;
    }
    break;
    
  case PARAMETER_TYPE_MATRIX:
    response.param.type = OpspaceParameter::PARAMETER_TYPE_MATRIX;
    {
      Matrix const * mm(param->getMatrix());
      if ( ! mm) {
	response.ok = false;
	response.errstr = "oops, looks like a bug: no matrix in matrix parameter";
	return true;
      }
      response.param.realval.resize(mm->rows() * mm->cols());
      response.param.nrows = mm->rows();
      response.param.ncols = mm->cols();
      Matrix::Map(&response.param.realval[0], mm->rows(), mm->cols()) = *mm;
    }
    break;
    
  default:
    response.ok = false;
    response.errstr = "unsupported or invalid type";
    return true;
  }
  
  response.ok = true;
  return true;
}
