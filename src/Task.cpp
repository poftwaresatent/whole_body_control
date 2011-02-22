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

#include <opspace/Task.hpp>

using namespace jspace;

namespace opspace {
  
  
  Parameter::
  Parameter(std::string const & name,
	    task_param_type_t type,
	    ParameterChecker const * checker)
    : name_(name),
      type_(type),
      checker_(checker)
  {
    switch (type) {
    case TASK_PARAM_TYPE_VOID:
    case TASK_PARAM_TYPE_INTEGER:
    case TASK_PARAM_TYPE_REAL:
    case TASK_PARAM_TYPE_VECTOR:
    case TASK_PARAM_TYPE_MATRIX:
      break;
    default:
      const_cast<task_param_type_t &>(type_) = TASK_PARAM_TYPE_VOID;
    }
  }
  
  
  Parameter::
  ~Parameter()
  {
  }
  
  
  void Parameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : void\n";
  }
  
  
  IntegerParameter::
  IntegerParameter(std::string const & name, ParameterChecker const * checker, int * integer)
    : Parameter(name, TASK_PARAM_TYPE_INTEGER, checker),
      integer_(integer)
  {
  }
  
  
  Status IntegerParameter::
  set(int integer)
  {
    Status st;
    if (checker_) {
      st = checker_->check(integer_, integer);
      if ( ! st) {
	return st;
      }
    }
    *integer_ = integer;
    return st;
  }
  
  
  void IntegerParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : integer = " << *integer_ << "\n";
  }


  RealParameter::
  RealParameter(std::string const & name, ParameterChecker const * checker, double * real)
    : Parameter(name, TASK_PARAM_TYPE_REAL, checker),
      real_(real)
  {
  }
  
  
  Status RealParameter::
  set(double real)
  {
    Status st;
    if (checker_) {
      st = checker_->check(real_, real);
      if ( ! st) {
	return st;
      }
    }
    *real_ = real;
    return st;
  }
  
  
  void RealParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : real = " << *real_ << "\n";
  }


  VectorParameter::
  VectorParameter(std::string const & name, ParameterChecker const * checker, Vector * vector)
    : Parameter(name, TASK_PARAM_TYPE_VECTOR, checker),
      vector_(vector)
  {
  }
  
  
  Status VectorParameter::
  set(Vector const & vector)
  {
    Status st;
    if (checker_) {
      st = checker_->check(vector_, vector);
      if ( ! st) {
	return st;
      }
    }
    *vector_ = vector;
    return st;
  }
  
  
  void VectorParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : vector =\n"
       << prefix << "  " << pretty_string(*vector_) << "\n";
  }


  MatrixParameter::
  MatrixParameter(std::string const & name, ParameterChecker const * checker, Matrix * matrix)
    : Parameter(name, TASK_PARAM_TYPE_MATRIX, checker),
      matrix_(matrix)
  {
  }
  
  
  Status MatrixParameter::
  set(Matrix const & matrix)
  {
    Status st;
    if (checker_) {
      st = checker_->check(matrix_, matrix);
      if ( ! st) {
	return st;
      }
    }
    *matrix_ = matrix;
    return st;
  }
  
  
  void MatrixParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : matrix =\n"
       << pretty_string(*matrix_, prefix + "  ") << "\n";
  }
  
  
  Task::
  Task(std::string const & name)
    : name_(name),
      sigma_threshold_(1.0e-2)
  {
    declareParameter("sigma_threshold", &sigma_threshold_);
  }
  
  
  Task::
  ~Task()
  {
    for (parameter_table_t::iterator ii(parameter_table_.begin());
	 ii != parameter_table_.end(); ++ii) {
      delete *ii;
    }
  }
  
  
  IntegerParameter * Task::
  declareParameter(std::string const & name, int * integer)
  {
    IntegerParameter * entry(new IntegerParameter(name, this, integer));
    parameter_table_.push_back(entry);
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  RealParameter * Task::
  declareParameter(std::string const & name, double * real)
  {
    RealParameter * entry(new RealParameter(name, this, real));
    parameter_table_.push_back(entry);
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  VectorParameter * Task::
  declareParameter(std::string const & name, Vector * vector)
  {
    VectorParameter * entry(new VectorParameter(name, this, vector));
    parameter_table_.push_back(entry);
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  MatrixParameter * Task::
  declareParameter(std::string const & name, Matrix * matrix)
  {
    MatrixParameter * entry(new MatrixParameter(name, this, matrix));
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
      (*ii)->dump(os, prefix + "    ");
    }
    pretty_print(actual_, os, prefix + "  actual:", prefix + "    ");
    pretty_print(command_, os, prefix + "  command:", prefix + "    ");
    pretty_print(jacobian_, os, prefix + "  Jacobian:", prefix + "    ");
  }
  
  
  Parameter * Task::
  lookupParameter(std::string const & name)
  {
    parameter_lookup_t::iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    return ii->second;
  }
  
  
  Parameter const * Task::
  lookupParameter(std::string const & name) const
  {
    parameter_lookup_t::const_iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    return ii->second;
  }
  
  
  Parameter * Task::
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
  
  
  Parameter const * Task::
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
  check(int const * param, int value) const
  {
    Status ok;
    return ok;
  }


  Status Task::
  check(double const * param, double value) const
  {
    Status ok;
    return ok;
  }

  
  Status Task::
  check(Vector const * param, Vector const & value) const
  {
    Status ok;
    return ok;
  }
  
  
  Status Task::
  check(Matrix const * param, Matrix const & value) const
  {
    Status ok; return ok;
  }


  void Task::
  dbg(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
  }
  
}
