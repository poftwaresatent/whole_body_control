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

#include <opspace/Parameter.hpp>

using namespace jspace;

namespace opspace {
  
  
  Parameter::
  Parameter(std::string const & name,
	    parameter_type_t type,
	    ParameterChecker const * checker)
    : name_(name),
      type_(type),
      checker_(checker)
  {
    switch (type) {
    case PARAMETER_TYPE_VOID:
    case PARAMETER_TYPE_INTEGER:
    case PARAMETER_TYPE_REAL:
    case PARAMETER_TYPE_VECTOR:
    case PARAMETER_TYPE_MATRIX:
      break;
    default:
      const_cast<parameter_type_t &>(type_) = PARAMETER_TYPE_VOID;
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
    : Parameter(name, PARAMETER_TYPE_INTEGER, checker),
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
    : Parameter(name, PARAMETER_TYPE_REAL, checker),
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
    : Parameter(name, PARAMETER_TYPE_VECTOR, checker),
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
    : Parameter(name, PARAMETER_TYPE_MATRIX, checker),
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
  
}
