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
  
  
  typedef enum {
    TASK_PARAM_SELECT_NONE =  0,
    TASK_PARAM_SELECT_GOAL =  1,
    TASK_PARAM_SELECT_KP   =  2,
    TASK_PARAM_SELECT_KD   =  4,
    TASK_PARAM_SELECT_VMAX =  8,
    TASK_PARAM_SELECT_AMAX = 16,
    TASK_PARAM_SELECT_ALL  = 31
  } task_param_select_t;
  
  
  class Task;
  
  
  class TaskParameter
  {
  public:
    Task const * owner_;
    std::string const name_;
    task_param_type_t const type_;
    size_t const index_;
    
    TaskParameter(Task const * owner,
		  std::string const & name,
		  task_param_type_t type,
		  size_t index);
    
    virtual ~TaskParameter();
    
    const int * getInteger() const   { return integer_; }
    const double * getReal() const   { return real_; }
    const Vector * getVector() const { return vector_; }
    const Matrix * getMatrix() const { return matrix_; }
    
    Status setInteger(int value);
    Status setReal(double value);
    Status setVector(Vector const & value);
    Status setMatrix(Matrix const & value);
    
    void initInteger(int value);
    void initReal(double value);
    void initVector(Vector const & value);
    void initMatrix(Matrix const & value);
    
    void dump(std::ostream & os, std::string const & title, std::string const & prefix) const;
    
  protected:
    int * integer_;
    double * real_;
    Vector * vector_;
    Matrix * matrix_;
  };
  
  
  class Task
  {
  protected:
    Task(std::string const & name,
	 task_param_select_t parameter_selection);
    
  public:
    typedef std::vector<TaskParameter *> parameter_table_t;
    
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
       current task state, the command, and the Jacobian. Given the
       current joint-space model passed as argument to this method,
       subclasses have to set the actual_, command_, and jacobian_
       fields. These will then get retrieved by the ServoBehavior
       instance to assemble joint torque commands according to the
       task hierarchy.
       
       \note Make sure your subclass sets the actual_, command_, and
       jacobian_ fields in the implementation of this method.
    */
    virtual Status update(Model const & model) = 0;
    
    Vector const & getActual() const   { return actual_; }
    Vector const & getCommand() const  { return command_; }
    Matrix const & getJacobian() const { return jacobian_; }
    
    TaskParameter * lookupParameter(std::string const & name);
    TaskParameter const * lookupParameter(std::string const & name) const;
    
    TaskParameter * lookupParameter(std::string const & name, task_param_type_t type);
    TaskParameter const * lookupParameter(std::string const & name, task_param_type_t type) const;
    
    virtual Status checkInteger(TaskParameter const * param, int value) const;
    virtual Status checkReal(TaskParameter const * param, double value) const;
    virtual Status checkVector(TaskParameter const * param, Vector const & value) const;
    virtual Status checkMatrix(TaskParameter const * param, Matrix const & value) const;
    
    parameter_table_t & getParameterTable()             { return parameter_table_; }
    parameter_table_t const & getParameterTable() const { return parameter_table_; }
    
    void dump(std::ostream & os, std::string const & title, std::string const & prefix) const;
    
  protected:
    typedef std::map<std::string, TaskParameter *> parameter_lookup_t;
    
    TaskParameter * defineParameter(std::string const & name,
				    task_param_type_t type);
    
    std::string const name_;
    Vector actual_;
    Vector command_;
    Matrix jacobian_;
    parameter_table_t parameter_table_;
    parameter_lookup_t parameter_lookup_;
    
    TaskParameter * goal_;
    TaskParameter * kp_;
    TaskParameter * kd_;
    TaskParameter * vmax_;
    TaskParameter * amax_;
  };
  
}

#endif // OPSPACE_TASK_HPP
