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

#ifndef OPSPACE_TASK_HPP
#define OPSPACE_TASK_HPP

#include <jspace/Model.hpp>
#include <jspace/Status.hpp>

namespace opspace {
  
  
  using jspace::Model;
  using jspace::Status;
  using jspace::Vector;
  using jspace::Matrix;
  
  
  typedef enum {
    TASK_PARAM_TYPE_VOID,	// no data (e.g. invalid type code)
    TASK_PARAM_TYPE_INTEGER,	// mapped to int
    TASK_PARAM_TYPE_REAL,	// mapped to double
    TASK_PARAM_TYPE_VECTOR,	// mapped to jspace::Vector
    TASK_PARAM_TYPE_MATRIX	// mapped to jspace::Matrix
  } task_param_type_t;
  
  
  class ParameterChecker {
  public:
    virtual ~ParameterChecker() {}
    virtual Status check(int const * param, int value) const = 0;
    virtual Status check(double const * param, double value) const = 0;
    virtual Status check(Vector const * param, Vector const & value) const = 0;
    virtual Status check(Matrix const * param, Matrix const & value) const = 0;
  };
  
  
  class Parameter
  {
  public:
    std::string const name_;
    task_param_type_t const type_;
    ParameterChecker const * checker_;
    
    Parameter(std::string const & name,
	      task_param_type_t type,
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
  
  
  class IntegerParameter : public Parameter {
  public:
    IntegerParameter(std::string const & name, ParameterChecker const * checker, int * integer);
    virtual int const * getInteger() const { return integer_; }
    virtual Status set(int integer);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    int * integer_;
  };
  
  
  class RealParameter : public Parameter {
  public:
    RealParameter(std::string const & name, ParameterChecker const * checker, double * real);
    virtual double const * getReal() const { return real_; }
    virtual Status set(double real);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    double * real_;
  };
  
  
  class VectorParameter : public Parameter {
  public:
    VectorParameter(std::string const & name, ParameterChecker const * checker, Vector * vector);
    virtual Vector const * getVector() const { return vector_; }
    virtual Status set(Vector const & vector);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    Vector * vector_;
  };
  
  
  class MatrixParameter : public Parameter {
  public:
    MatrixParameter(std::string const & name, ParameterChecker const * checker, Matrix * matrix);
    virtual Matrix const * getMatrix() const { return matrix_; }
    virtual Status set(Matrix const & matrix);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    Matrix * matrix_;
  };
  
  
  class Task
    : public ParameterChecker
  {
  protected:
    explicit Task(std::string const & name);
    
  public:
    typedef std::vector<Parameter *> parameter_table_t;
    
    virtual ~Task();
    
    /**
       Abstract, implemented by subclasses in order to initialize the
       task. This is important for stateful tasks, for instance in
       order to initialize a trajectory-following behavior. The init()
       method also gets called when tasks are switched at runtime, so
       subclasses should NOT assume that init() only gets called once
       at startup.
    */
    virtual Status init(Model const & model) = 0;
    
    /**
       Abstract, implemented by subclasses in order to compute the
       current task state, the command acceleration, and the
       Jacobian. Given the current joint-space model passed as
       argument to this method, subclasses have to set the actual_,
       command_, and jacobian_ fields. These will then get retrieved
       according to the task hierarchy and assembled into joint torque
       commands using dynamically consistent nullspace projection.
       
       \note Make sure your subclass sets the actual_, command_, and
       jacobian_ fields in the implementation of this method.
    */
    virtual Status update(Model const & model) = 0;
    
    std::string const & getName() const { return name_; }
    
    Vector const & getActual() const   { return actual_; }
    Vector const & getCommand() const  { return command_; }
    Matrix const & getJacobian() const { return jacobian_; }
    
    Parameter * lookupParameter(std::string const & name);
    Parameter const * lookupParameter(std::string const & name) const;
    
    Parameter * lookupParameter(std::string const & name, task_param_type_t type);
    Parameter const * lookupParameter(std::string const & name, task_param_type_t type) const;
    
    virtual Status check(int const * param, int value) const;
    virtual Status check(double const * param, double value) const;
    virtual Status check(Vector const * param, Vector const & value) const;
    virtual Status check(Matrix const * param, Matrix const & value) const;
    
    parameter_table_t & getParameterTable()             { return parameter_table_; }
    parameter_table_t const & getParameterTable() const { return parameter_table_; }
    
    void dump(std::ostream & os, std::string const & title, std::string const & prefix) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    typedef std::map<std::string, Parameter *> parameter_lookup_t;
    
    IntegerParameter * declareParameter(std::string const & name, int * integer);
    RealParameter * declareParameter(std::string const & name, double * real);
    VectorParameter * declareParameter(std::string const & name, Vector * vector);
    MatrixParameter * declareParameter(std::string const & name, Matrix * matrix);
    
    std::string const name_;
    Vector actual_;
    Vector command_;
    Matrix jacobian_;
    parameter_table_t parameter_table_;
    parameter_lookup_t parameter_lookup_;
  };
  
}

#endif // OPSPACE_TASK_HPP
