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

#include <Task.hpp>

using namespace jspace;

namespace opspace {
  
  
  TaskParameter::
  TaskParameter(Task const * owner,
		std::string const & name,
		task_param_type_t type,
		size_t index)
    : owner_(owner),
      name_(name),
      type_(type),
      index_(index),
      integer_(0),
      real_(0),
      vector_(0),
      matrix_(0)
  {
    switch (type) {
    case TASK_PARAM_TYPE_INTEGER:
      integer_ = new int();
      break;
    case TASK_PARAM_TYPE_REAL:
      real_ = new double();
      break;
    case TASK_PARAM_TYPE_VECTOR:
      vector_ = new Vector();
      break;
    case TASK_PARAM_TYPE_MATRIX:
      matrix_ = new Matrix();
      break;
    default:
      const_cast<task_param_type_t &>(type_) = TASK_PARAM_TYPE_VOID;
    }
  }
  
  
  TaskParameter::
  ~TaskParameter()
  {
    delete integer_;
    delete real_;
    delete vector_;
    delete matrix_;
  }
  
  
  Status TaskParameter::
  setInteger(int value)
  {
    Status st;
    if (owner_) {
      st = owner_->checkInteger(this, value);
      if ( ! st) {
	return st;
      }
    }
    *integer_ = value;
    return st;
  }
  
  
  Status TaskParameter::
  setReal(double value)
  {
    Status st;
    if (owner_) {
      st = owner_->checkReal(this, value);
      if ( ! st) {
	return st;
      }
    }
    *real_ = value;
    return st;
  }
  
  
  Status TaskParameter::
  setVector(Vector const & value)
  {
    Status st;
    if (owner_) {
      st = owner_->checkVector(this, value);
      if ( ! st) {
	return st;
      }
    }
    *vector_ = value;
    return st;
  }
  
  
  Status TaskParameter::
  setMatrix(Matrix const & value)
  {
    Status st;
    if (owner_) {
      st = owner_->checkMatrix(this, value);
      if ( ! st) {
	return st;
      }
    }
    *matrix_ = value;
    return st;
  }
  
  
  void TaskParameter::
  initInteger(int value)
  {
    *integer_ = value;
  }
  
  
  void TaskParameter::
  initReal(double value)
  {
    *real_ = value;
  }
  
  
  void TaskParameter::
  initVector(Vector const & value)
  {
    *vector_ = value;
  }
  
  
  void TaskParameter::
  initMatrix(Matrix const & value)
  {
    *matrix_ = value;
  }
  
  
  void TaskParameter::
  dump(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    switch (type_) {
    case TASK_PARAM_TYPE_INTEGER:
      os << prefix << name_ << " : integer = " << *integer_ << "\n";
      break;
    case TASK_PARAM_TYPE_REAL:
      os << prefix << name_ << " : real = " << *real_ << "\n";
      break;
    case TASK_PARAM_TYPE_VECTOR:
      os << prefix << name_ << " : vector = " << pretty_string(*vector_) << "\n";
      break;
    case TASK_PARAM_TYPE_MATRIX:
      os << prefix << name_ << " : matrix =\n" << pretty_string(*matrix_, prefix + "  ") << "\n";
      break;
    default:
      os << prefix << name_ << " : void\n";
    }
  }
  
  
  Task::
  Task(std::string const & name,
       task_param_select_t parameter_selection)
    : name_(name)
  {
    if (parameter_selection & TASK_PARAM_SELECT_GOAL) {
      goal_ = defineParameter("goal", TASK_PARAM_TYPE_VECTOR);
    }
    if (parameter_selection & TASK_PARAM_SELECT_KP) {
      kp_ = defineParameter("kp", TASK_PARAM_TYPE_VECTOR);
    }
    if (parameter_selection & TASK_PARAM_SELECT_KD) {
      kd_ = defineParameter("kd", TASK_PARAM_TYPE_VECTOR);
    }
    if (parameter_selection & TASK_PARAM_SELECT_VMAX) {
      vmax_ = defineParameter("vmax", TASK_PARAM_TYPE_VECTOR);
    }
    if (parameter_selection & TASK_PARAM_SELECT_AMAX) {
      amax_ = defineParameter("amax", TASK_PARAM_TYPE_VECTOR);
    }
  }
  
  
  Task::
  ~Task()
  {
    for (parameter_table_t::iterator ii(parameter_table_.begin());
	 ii != parameter_table_.end(); ++ii) {
      delete *ii;
    }
  }
  
  
  TaskParameter * Task::
  defineParameter(std::string const & name,
		  task_param_type_t type)
  {
    size_t const index(parameter_table_.size());
    TaskParameter * entry(new TaskParameter(this, name, type, index));
    parameter_table_.push_back(entry);
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  void Task::
  dump(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "task: `" << name_ << "'\n"
       << prefix << "  parameters:\n";
    for (parameter_table_t::const_iterator ii(parameter_table_.begin());
	 ii != parameter_table_.end(); ++ii) {
      (*ii)->dump(os, "", prefix + "    ");
    }
    pretty_print(actual_, os, "    actual:", "      ");
    pretty_print(command_, os, "    command:", "      ");
    pretty_print(Jacobian_, os, "    Jacobian:", "      ");
  }
  
  
  TaskParameter * Task::
  lookupParameter(std::string const & name)
  {
    parameter_lookup_t::iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    return ii->second;
  }
  
  
  TaskParameter const * Task::
  lookupParameter(std::string const & name) const
  {
    parameter_lookup_t::const_iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    return ii->second;
  }
  
  
  TaskParameter * Task::
  lookupParameter(std::string const & name, task_param_type_t type)
  {
    parameter_lookup_t::iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    if (type != ii->second->type_) {
      return 0;	      // could maybe implement some sort of casting...
    }
    return ii->second;
  }
  
  
  TaskParameter const * Task::
  lookupParameter(std::string const & name, task_param_type_t type) const
  {
    parameter_lookup_t::const_iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    if (type != ii->second->type_) {
      return 0;	      // could maybe implement some sort of casting...
    }
    return ii->second;
  }
  
  
  Status Task::
  checkInteger(TaskParameter const * param, int value) const
  {
    Status ok;
    return ok;
  }
  
  
  Status Task::
  checkReal(TaskParameter const * param, double value) const
  {
    Status ok;
    return ok;
  }
  
  
  Status Task::
  checkVector(TaskParameter const * param, Vector const & value) const
  {
    Status ok;
    return ok;
  }
  
  
  Status Task::
  checkMatrix(TaskParameter const * param, Matrix const & value) const
  {
    Status ok;
    return ok;
  }
  
}
