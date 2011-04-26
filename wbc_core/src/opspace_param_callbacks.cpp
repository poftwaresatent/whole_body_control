/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
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

#include <wbc_core/opspace_param_callbacks.hpp>
#include <wbc_msgs/ChannelFeedback.h>

using namespace opspace;
using namespace wbc_msgs;
using namespace boost;
using namespace std;


namespace wbc_core_opspace {
  
  
  ParamCallbacks::
  ParamCallbacks()
    : next_channel_id_(0)
  {
  }
  
  
  void ParamCallbacks::
  init(ros::NodeHandle node,
       boost::shared_ptr<opspace::ReflectionRegistry> registry,
       size_t input_queue_size,
       size_t output_queue_size)
    throw(std::runtime_error)
  {
    if (registry_) {
      throw runtime_error("already initialized");
    }
    registry_ = registry;
    
    set_param_ = node.advertiseService("set_param",
				       &::wbc_core_opspace::ParamCallbacks::setParam,
				       this);
    get_param_ = node.advertiseService("get_param",
				       &::wbc_core_opspace::ParamCallbacks::getParam,
				       this);
    list_params_ = node.advertiseService("list_params",
					 &::wbc_core_opspace::ParamCallbacks::listParams,
					 this);
    open_channel_ = node.advertiseService("open_channel",
					  &::wbc_core_opspace::ParamCallbacks::openChannel,
					  this);
    string_sub_ = node.subscribe("string_channel",
				 input_queue_size,
				 &::wbc_core_opspace::ParamCallbacks::stringChannel,
				 this);
    integer_sub_ = node.subscribe("integer_channel",
				  input_queue_size,
				  &::wbc_core_opspace::ParamCallbacks::integerChannel,
				  this);
    real_sub_ = node.subscribe("real_channel",
			       input_queue_size,
			       &::wbc_core_opspace::ParamCallbacks::realChannel,
			       this);
    vector_sub_ = node.subscribe("vector_channel",
				 input_queue_size,
				 &::wbc_core_opspace::ParamCallbacks::vectorChannel,
				 this);
    matrix_sub_ = node.subscribe("matrix_channel",
				 input_queue_size,
				 &::wbc_core_opspace::ParamCallbacks::matrixChannel,
				 this);
    channel_feedback_ = node.advertise<wbc_msgs::ChannelFeedback>("channel_feedback",
								  output_queue_size);
  }
  
  
  opspace::Parameter * ParamCallbacks::
  findParam(std::string const & com_type,
	    std::string const & com_name,
	    std::string const & param_name,
	    std::string & errstr)
  {
    if ( ! registry_) {
      errstr = "not initialized";
      return 0;
    }
    opspace::Parameter * param(registry_->lookupParameter(com_type, com_name, param_name));
    if ( ! param) {
      errstr = com_type + "/" + com_name + "/" + param_name + " not found";
    }
    return param;
  }
  
  
  bool ParamCallbacks::
  setParam(wbc_msgs::SetParameter::Request & request,
	   wbc_msgs::SetParameter::Response & response)
  {
    Status status;
    Parameter * param(findParam(request.com_type,
				request.com_name,
				request.param.name,
				status.errstr));
    if ( ! param) {
      status.ok = false;
      // status.errstr set by findParam...
      return true;
    }
    
    switch (request.param.type) {
      
    case OpspaceParameter::PARAMETER_TYPE_STRING:
      status = param->set(request.param.strval);
      break;
      
    case OpspaceParameter::PARAMETER_TYPE_INTEGER:
      // grr, one day the 32 vs 64 bit thing will bite us
      status = param->set((int) request.param.intval);
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
	// I tried to find a more or less elegant and non-intrusive
	// way to do without the tmp, and failed. Eigen is simply not
	// made to mix well with runtime dynamic typing. For instance,
	// Eigen::Map<> can be made to look "just like"
	// Eigen::Matrix<> to the compiler in expressions, but trying
	// to pass a const ref to a map in some place which expects a
	// const ref to a vector simply does not work.
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
  
  
  static bool param_to_msg(Parameter const * param,
			   wbc_msgs::OpspaceParameter & msg)
  {
    msg.name = param->name_;
    msg.type = OpspaceParameter::PARAMETER_TYPE_VOID;
    
    switch (param->type_) {
      
    case PARAMETER_TYPE_STRING:
      if ( ! param->getString()) {
	return false;
      }
      msg.type = OpspaceParameter::PARAMETER_TYPE_STRING;
      msg.strval = *param->getString();
      break;
      
    case PARAMETER_TYPE_INTEGER:
      if ( ! param->getInteger()) {
	return false;
      }
      msg.type = OpspaceParameter::PARAMETER_TYPE_INTEGER;
      msg.intval = *param->getInteger();
      break;
      
    case PARAMETER_TYPE_REAL:
      if ( ! param->getReal()) {
	return false;
      }
      msg.type = OpspaceParameter::PARAMETER_TYPE_REAL;
      msg.realval.resize(1);
      msg.realval[0] = *param->getReal();
      break;
      
    case PARAMETER_TYPE_VECTOR:
      if ( ! param->getVector()) {
	return false;
      }
      msg.type = OpspaceParameter::PARAMETER_TYPE_VECTOR;
      {
	Vector const * vv(param->getVector());
	msg.realval.resize(vv->rows());
	Vector::Map(&msg.realval[0], vv->rows()) = *vv;
      }
      break;
      
    case PARAMETER_TYPE_MATRIX:
      if ( ! param->getMatrix()) {
	return false;
      }
      msg.type = OpspaceParameter::PARAMETER_TYPE_MATRIX;
      {
	Matrix const * mm(param->getMatrix());
	msg.realval.resize(mm->rows() * mm->cols());
	msg.nrows = mm->rows();
	msg.ncols = mm->cols();
	Matrix::Map(&msg.realval[0], mm->rows(), mm->cols()) = *mm;
      }
      break;
      
    default:
      return false;
    }
    
    return true;
  }
  
  
  bool ParamCallbacks::
  getParam(wbc_msgs::GetParameter::Request & request,
	   wbc_msgs::GetParameter::Response & response)
  {
    Status status;
    Parameter const * param(findParam(request.com_type,
				      request.com_name,
				      request.param_name,
				      status.errstr));
    if ( ! param) {
      status.ok = false;
      // status.errstr set by findParam...
      return true;
    }
    
    if ( ! param_to_msg(param, response.param)) {
      status.ok = false;
      status.errstr = "parameter conversion error (probably a bug!)";
      return true;
    }
    
    response.ok = true;
    return true;
  }
  
  
  bool ParamCallbacks::
  listParams(wbc_msgs::ListParameters::Request & request,
	     wbc_msgs::ListParameters::Response & response)
  {
    if ( ! registry_) {
      response.ok = false;
      response.errstr = "not initialized";
      return true;
    }
    
    ReflectionRegistry::enumeration_t enumeration;
    registry_->enumerate(enumeration);
    for (size_t ii(0); ii < enumeration.size(); ++ii) {
      OpspaceParameter msg;
      if (param_to_msg(enumeration[ii].parameter, msg)) { // should "never" fail though...
	response.com_type.push_back(enumeration[ii].type_name);
	response.com_name.push_back(enumeration[ii].instance_name);
	response.param.push_back(msg);
      }
    }
    
    response.ok = true;
    return true;
  }
  
  
  bool ParamCallbacks::
  openChannel(wbc_msgs::OpenChannel::Request & request,
	      wbc_msgs::OpenChannel::Response & response)
  {
    response.param_type = OpspaceParameter::PARAMETER_TYPE_VOID;
    Parameter * param(findParam(request.com_type,
				request.com_name,
				request.param_name,
				response.errstr));
    if ( ! param) {
      response.ok = false;
      // status.errstr set by findParam...
      return true;
    }
    
    switch (param->type_) {
      
    case PARAMETER_TYPE_STRING:
      if (( ! param->getString())
	  || ( ! dynamic_cast<StringParameter*>(param))) {
	response.ok = false;
	response.errstr = "buggy string parameter (oops!)";
	return true;
      }
      else {
	StringChannel msg;
	msg.channel_id = next_channel_id_++;
	msg.transaction_id = 0;
	msg.value = *param->getString();
	response.ok = true;
	response.string_channel.push_back(msg);
	strings_.insert(make_pair(msg.channel_id, dynamic_cast<StringParameter*>(param)));
	return true;
      }
      
    case PARAMETER_TYPE_INTEGER:
      if (( ! param->getInteger())
	  || ( ! dynamic_cast<IntegerParameter*>(param))) {
	response.ok = false;
	response.errstr = "buggy integer parameter (oops!)";
	return true;
      }
      else {
	IntegerChannel msg;
	msg.channel_id = next_channel_id_++;
	msg.transaction_id = 0;
	msg.value = *param->getInteger();
	response.ok = true;
	response.integer_channel.push_back(msg);
	integers_.insert(make_pair(msg.channel_id, dynamic_cast<IntegerParameter*>(param)));
	return true;
      }
      
    case PARAMETER_TYPE_REAL:
      if (( ! param->getReal())
	  || ( ! dynamic_cast<RealParameter*>(param))) {
	response.ok = false;
	response.errstr = "buggy real parameter (oops!)";
	return true;
      }
      else {
	RealChannel msg;
	msg.channel_id = next_channel_id_++;
	msg.transaction_id = 0;
	msg.value = *param->getReal();
	response.ok = true;
	response.real_channel.push_back(msg);
	reals_.insert(make_pair(msg.channel_id, dynamic_cast<RealParameter*>(param)));
	return true;
      }
      
    case PARAMETER_TYPE_VECTOR:
      if (( ! param->getVector())
	  || ( ! dynamic_cast<VectorParameter*>(param))) {
	response.ok = false;
	response.errstr = "buggy vector parameter (oops!)";
	return true;
      }
      else {
	VectorChannel msg;
	msg.channel_id = next_channel_id_++;
	msg.transaction_id = 0;
	Vector const * vv(param->getVector());
	msg.value.resize(vv->rows());
	Vector::Map(&msg.value[0], vv->rows()) = *vv;
	response.ok = true;
	response.vector_channel.push_back(msg);
	vectors_.insert(make_pair(msg.channel_id, dynamic_cast<VectorParameter*>(param)));
	return true;
      }
      
    case PARAMETER_TYPE_MATRIX:
      if (( ! param->getMatrix())
	  || ( ! dynamic_cast<MatrixParameter*>(param))) {
	response.ok = false;
	response.errstr = "buggy matrix parameter (oops!)";
	return true;
      }
      else {
	MatrixChannel msg;
	msg.channel_id = next_channel_id_++;
	msg.transaction_id = 0;
	Matrix const * mm(param->getMatrix());
	msg.value.resize(mm->rows() * mm->cols());
	msg.nrows = mm->rows();
	msg.ncols = mm->cols();
	Matrix::Map(&msg.value[0], mm->rows(), mm->cols()) = *mm;
	response.ok = true;
	response.matrix_channel.push_back(msg);
	matrices_.insert(make_pair(msg.channel_id, dynamic_cast<MatrixParameter*>(param)));
	return true;
      }
      
      //    default:
    }
    
    response.ok = false;
    response.errstr = "buggy parameter type handling (oops!)";
    return true;
  }
  
  
  void ParamCallbacks::
  stringChannel(wbc_msgs::StringChannel const & msg)
  {
    ChannelFeedback feedback;
    feedback.ok = true;
    feedback.channel_id = msg.channel_id;
    feedback.transaction_id = msg.transaction_id;
    
    std::map<int, opspace::StringParameter *>::iterator ip(strings_.find(msg.channel_id));
    if (strings_.end() == ip) {
      feedback.ok = false;
      feedback.errstr = "invalid string channel";
    }
    
    if (feedback.ok) {
      Status const status(ip->second->set(msg.value));
      if ( ! status) {
	feedback.ok = false;
	feedback.errstr = status.errstr;
      }
    }
    
    channel_feedback_.publish(feedback);
  }
  
  
  void ParamCallbacks::
  integerChannel(wbc_msgs::IntegerChannel const & msg)
  {
    ChannelFeedback feedback;
    feedback.ok = true;
    feedback.channel_id = msg.channel_id;
    feedback.transaction_id = msg.transaction_id;
    
    std::map<int, opspace::IntegerParameter *>::iterator ip(integers_.find(msg.channel_id));
    if (integers_.end() == ip) {
      feedback.ok = false;
      feedback.errstr = "invalid integer channel";
    }
    
    if (feedback.ok) {
      Status const status(ip->second->set(msg.value));
      if ( ! status) {
	feedback.ok = false;
	feedback.errstr = status.errstr;
      }
    }
    
    channel_feedback_.publish(feedback);
  }

  
  void ParamCallbacks::
  realChannel(wbc_msgs::RealChannel const & msg)
  {
    ChannelFeedback feedback;
    feedback.ok = true;
    feedback.channel_id = msg.channel_id;
    feedback.transaction_id = msg.transaction_id;
    
    std::map<int, opspace::RealParameter *>::iterator ip(reals_.find(msg.channel_id));
    if (reals_.end() == ip) {
      feedback.ok = false;
      feedback.errstr = "invalid real channel";
    }
    
    if (feedback.ok) {
      Status const status(ip->second->set(msg.value));
      if ( ! status) {
	feedback.ok = false;
	feedback.errstr = status.errstr;
      }
    }
    
    channel_feedback_.publish(feedback);
  }

  
  void ParamCallbacks::
  vectorChannel(wbc_msgs::VectorChannel const & msg)
  {
    ChannelFeedback feedback;
    feedback.ok = true;
    feedback.channel_id = msg.channel_id;
    feedback.transaction_id = msg.transaction_id;
    
    std::map<int, opspace::VectorParameter *>::iterator ip(vectors_.find(msg.channel_id));
    if (vectors_.end() == ip) {
      feedback.ok = false;
      feedback.errstr = "invalid vector channel";
    }
    
    if (feedback.ok) {
      Vector tmp(jspace::Vector::Map(&msg.value[0], msg.value.size()));
      Status const status(ip->second->set(tmp));
      if ( ! status) {
	feedback.ok = false;
	feedback.errstr = status.errstr;
      }
    }
    
    channel_feedback_.publish(feedback);
  }
  
  
  void ParamCallbacks::
  matrixChannel(wbc_msgs::MatrixChannel const & msg)
  {
    ChannelFeedback feedback;
    feedback.ok = true;
    feedback.channel_id = msg.channel_id;
    feedback.transaction_id = msg.transaction_id;
    
    std::map<int, opspace::VectorParameter *>::iterator ip(vectors_.find(msg.channel_id));
    if (vectors_.end() == ip) {
      feedback.ok = false;
      feedback.errstr = "invalid vector channel";
    }
    
    if (feedback.ok) {
      Matrix tmp(jspace::Vector::Map(&msg.value[0], msg.nrows, msg.ncols));
      Status const status(ip->second->set(tmp));
      if ( ! status) {
	feedback.ok = false;
	feedback.errstr = status.errstr;
      }
    }
    
    channel_feedback_.publish(feedback);
  }
  
}
