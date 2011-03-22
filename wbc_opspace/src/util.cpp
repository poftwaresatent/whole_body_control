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

#include <wbc_opspace/util.h>

using namespace opspace;
using namespace wbc_msgs;
using namespace boost;
using namespace std;


namespace wbc_opspace {
  
  
  void ParamCallbacks::
  init(ros::NodeHandle node,
       std::string const & set_param_service_name,
       std::string const & get_param_service_name,
       boost::shared_ptr<opspace::Factory> factory,
       boost::shared_ptr<opspace::ControllerNG> controller) throw(std::runtime_error)
  {
    if (factory_) {
      throw runtime_error("already initialized");
    }
    if ( ! factory) {
      throw runtime_error("null factory");
    }
    if ( ! controller) {
      throw runtime_error("null controller");
    }
    factory_ = factory;
    controller_ = controller;
    set_param_ = node.advertiseService(set_param_service_name,
				       &::wbc_opspace::ParamCallbacks::setParam,
				       this);
    get_param_ = node.advertiseService(get_param_service_name,
				       &::wbc_opspace::ParamCallbacks::getParam,
				       this);
  }
  
  
  opspace::Parameter * ParamCallbacks::
  findParam(std::string const & com_type,
	    std::string const & com_name,
	    std::string const & param_name,
	    std::string & errstr)
  {
    if ( ! factory_) {
      errstr = "not initialized";
      return 0;
    }
    
    if ("task" == com_type) {
      shared_ptr<Task> task(factory_->findTask(com_name));
      if ( ! task) {
	errstr = "no such task";
	return 0;
      }
      errstr = "trying task parameters...";
      return task->lookupParameter(param_name);
    }
    
    if ("skill" == com_type) {
      shared_ptr<Behavior> skill(factory_->findBehavior(com_name));
      if ( ! skill) {
	errstr = "no such skill";
	return 0;
      }
      errstr = "trying skill parameters...";
      return skill->lookupParameter(param_name);
    }
    
    if ("servo" == com_type) {
      // ignore com_name for now...
      errstr = "trying servo parameters...";
      return controller_->lookupParameter(param_name);
    }
    
    errstr = "invalid com_type (use task, skill, or servo)";
    return 0;
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
      // status.errstr set by find_param...
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
  
}
