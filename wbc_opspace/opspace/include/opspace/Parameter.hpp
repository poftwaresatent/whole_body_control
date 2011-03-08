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

#ifndef OPSPACE_PARAMETER_HPP
#define OPSPACE_PARAMETER_HPP

#include <jspace/Status.hpp>
#include <jspace/wrap_eigen.hpp>

namespace opspace {
  
  
  using jspace::Status;
  using jspace::Vector;
  using jspace::Matrix;
  
  
  /**
     Enumeration type for task parameter types.
  */
  typedef enum {
    PARAMETER_TYPE_VOID,	//!< no data (e.g. invalid type code)
    PARAMETER_TYPE_INTEGER,	//!< mapped to int
    PARAMETER_TYPE_REAL,	//!< mapped to double
    PARAMETER_TYPE_VECTOR,	//!< mapped to jspace::Vector
    PARAMETER_TYPE_MATRIX	//!< mapped to jspace::Matrix
  } parameter_type_t;
  
  
  /**
     Interface for classes that check parameters.  Classes that
     inherit this interface can be passed to Parameter constructors
     and will then be called back by that Parameter instance's set()
     methods.
  */
  class ParameterChecker {
  public:
    virtual ~ParameterChecker() {}
    virtual Status check(int const * param, int value) const = 0;
    virtual Status check(double const * param, double value) const = 0;
    virtual Status check(Vector const * param, Vector const & value) const = 0;
    virtual Status check(Matrix const * param, Matrix const & value) const = 0;
  };
  
  
  /**
     Abstract base for all (task) parameters. Fairly minimal for now:
     parameters have a name, a type, and (optionally) an associated
     checker. Ideas for future extensions are e.g. documentation
     strings, optional bounds for automatic checks, and a more generic
     type interface (instead of relying on parameter_type_t).
     
     \note This base class can be instantiated, but it just behaves
     like a PARAMETER_TYPE_VOID parameter: you cannot get or set
     anything.
  */
  class Parameter
  {
  public:
    std::string const name_;
    parameter_type_t const type_;
    ParameterChecker const * checker_;
    
    Parameter(std::string const & name,
	      parameter_type_t type,
	      ParameterChecker const * checker);
    
    virtual ~Parameter();
    
    virtual int const * getInteger() const   { return 0; }
    virtual double const * getReal() const   { return 0; }
    virtual Vector const * getVector() const { return 0; }
    virtual Matrix const * getMatrix() const { return 0; }
    
    virtual Status set(int integer) { Status err(false, "type mismatch"); return err; }
    virtual Status set(double real) { Status err(false, "type mismatch"); return err; }
    virtual Status set(Vector const & vector) { Status err(false, "type mismatch"); return err; }
    virtual Status set(Matrix const & matrix) { Status err(false, "type mismatch"); return err; }
    
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  };
  
  
  /** Implementation for integer parameters: a single int value. */
  class IntegerParameter : public Parameter {
  public:
    IntegerParameter(std::string const & name, ParameterChecker const * checker, int * integer);
    virtual int const * getInteger() const { return integer_; }
    virtual Status set(int integer);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    int * integer_;
  };
  
  
  /** Implementation for real parameters: a single double value. */
  class RealParameter : public Parameter {
  public:
    RealParameter(std::string const & name, ParameterChecker const * checker, double * real);
    virtual double const * getReal() const { return real_; }
    virtual Status set(double real);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    double * real_;
  };
  
  
  /** Implementation for vector parameters: a vector of double values. */
  class VectorParameter : public Parameter {
  public:
    VectorParameter(std::string const & name, ParameterChecker const * checker, Vector * vector);
    virtual Vector const * getVector() const { return vector_; }
    virtual Status set(Vector const & vector);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    Vector * vector_;
  };
  
  
  /** Implementation for matrix parameters: a matrix of double values. */
  class MatrixParameter : public Parameter {
  public:
    MatrixParameter(std::string const & name, ParameterChecker const * checker, Matrix * matrix);
    virtual Matrix const * getMatrix() const { return matrix_; }
    virtual Status set(Matrix const & matrix);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    Matrix * matrix_;
  };
  
}

#endif // OPSPACE_PARAMETER_HPP
