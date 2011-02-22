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
  
  
  /**
     Enumeration type for task parameter types.
  */
  typedef enum {
    TASK_PARAM_TYPE_VOID,	//!< no data (e.g. invalid type code)
    TASK_PARAM_TYPE_INTEGER,	//!< mapped to int
    TASK_PARAM_TYPE_REAL,	//!< mapped to double
    TASK_PARAM_TYPE_VECTOR,	//!< mapped to jspace::Vector
    TASK_PARAM_TYPE_MATRIX	//!< mapped to jspace::Matrix
  } task_param_type_t;
  
  
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
     type interface (instead of relying on task_param_type_t).
     
     \note This base class can be instantiated, but it just behaves
     like a TASK_PARAM_TYPE_VOID parameter: you cannot get or set
     anything.
  */
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
  
  
  /**
     Partially abstract base class for all operational space
     tasks. The base class provides parameter introspection facilities
     and an interface that is used by opspace::Controller instances to
     come up with control signals for a robot.
     
     This is one of the most important classes in the opspace
     namespace. You implement tasks by subclassing from it (or one of
     its more specialized derivatives) and implementing the init() and
     update() methods. You can also override some other methods if you
     are not happy with the defaults.
     
     A simple but complete example of concrete Task subclass would
     look like this:
     
     \code
  class Thermostat : public Task {
  public:
    Thermostat(std::string const & name) : Task(name), temp_(0) {
      declareParameter("desired_temperature", &temp_);
    }
    
    virtual Status init(Model const & model) {
      if (1 != model.getNDOF()) {
	return Status(false, "are you sure this is a fridge?");
      }
      jacobian_ = Vector::Ones(1); // somewhat spurious in this example
      command_ = Vector::Zero(1);
      return update(model);
    }
    
    virtual Status update(Model const & model) {
      actual_ = model.getState().position_;
      if (actual_[0] > temp_) {
	command_[0] = 1;
      }
      else {
	command_[0] = 0;
      }
      // jacobian_ was set in init() and never changes in this example
      Status ok;
      return ok;
    }
    
  private:
    double temp_;
  };
     \endcode
     
  */
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
    
    /**
       \return The actual "position" of the robot in this task
       space. Reminder: actual_ must be set by subclasses in their
       update() method.
    */
    Vector const & getActual() const   { return actual_; }
    
    /**
       \return The command of this task. In the operational space
       formulation, this is simply the desired acceleration of the
       task point (in task space, of course). Reminder: command_ must
       be set by subclasses in their update() method.
    */
    Vector const & getCommand() const  { return command_; }
    
    /**
       \return The current Jacobian of this task space. The Jacobian
       maps joint velocities to task space velocities (it thus has M
       rows and N columns, where M is the dimension of the task space
       and N the number of degrees of freedom of the robot). Usually,
       the Jacobian is configuration dependent and is updated along
       with the command in the update() method. Some tasks, however,
       have simple and constant Jacobians which can be set in the
       init() method. The Jacobian is used by the opspace::Controller
       to compensate for rigid body dynamics and to decouple tasks
       according to a strict hierarchy. Reminder: jacobian_ must be
       set by subclasses in their update() method.
    */
    Matrix const & getJacobian() const { return jacobian_; }
    
    /**
       \return A pointer to the Parameter subclass instance which
       represents a certain named parameter of the task, or zero if
       the name does not match any parameter. You can use
       getParameterTable() to inspect the list of parameters.
    */
    Parameter * lookupParameter(std::string const & name);

    /**
       \return A const pointer to the Parameter subclass instance
       which represents a certain named parameter of the task, or zero
       if the name does not match any parameter. You can use
       getParameterTable() to inspect the list of parameters.
    */
    Parameter const * lookupParameter(std::string const & name) const;
    
    /**
       \return A pointer to the Parameter subclass instance which
       represents a certain named parameter of the task AND matches
       the given type, or zero if the name or the type does not
       match. You can use getParameterTable() to inspect the list of
       parameters.
    */
    Parameter * lookupParameter(std::string const & name, task_param_type_t type);

    /**
       \return A const pointer to the Parameter subclass instance
       which represents a certain named parameter of the task AND
       matches the given type, or zero if the name or the type does
       not match. You can use getParameterTable() to inspect the list
       of parameters.
    */
    Parameter const * lookupParameter(std::string const & name, task_param_type_t type) const;
    
    /** Default implementation for ParameterChecker. Always returns succes. */
    virtual Status check(int const * param, int value) const;
    
    /** Default implementation for ParameterChecker. Always returns succes. */
    virtual Status check(double const * param, double value) const;
    
    /** Default implementation for ParameterChecker. Always returns succes. */
    virtual Status check(Vector const * param, Vector const & value) const;
    
    /** Default implementation for ParameterChecker. Always returns succes. */
    virtual Status check(Matrix const * param, Matrix const & value) const;
    
    parameter_table_t & getParameterTable()             { return parameter_table_; }
    parameter_table_t const & getParameterTable() const { return parameter_table_; }

    /**
       SVD cutoff value for pseudo inverse, exists in all tasks
       because Controller implementations need it.
    */
    double getSigmaThreshold() const { return sigma_threshold_; }
    
    void dump(std::ostream & os, std::string const & title, std::string const & prefix) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    typedef std::map<std::string, Parameter *> parameter_lookup_t;
    
    /**
       Used by subclasseto make one of their fields accessible to the
       outside via the task parameter table. The parameter then
       becomes available through one of the lookupParameter() and
       getParameterTable() methods.
       
       \note Everyone is granted read access to the parameter. Write
       access is protected in two ways: the caller needs to have a
       non-const handle on the Task instance, and the Parameter::set()
       method will call Task::check() before actually writing a new
       value into the pointer provided here at declaration time.
    */
    IntegerParameter * declareParameter(std::string const & name, int * integer);
    
    /** See also declareParameter(std::string const &, int *)... */
    RealParameter * declareParameter(std::string const & name, double * real);

    /** See also declareParameter(std::string const &, int *)... */
    VectorParameter * declareParameter(std::string const & name, Vector * vector);

    /** See also declareParameter(std::string const &, int *)... */
    MatrixParameter * declareParameter(std::string const & name, Matrix * matrix);
    
    std::string const name_;
    Vector actual_;
    Vector command_;
    Matrix jacobian_;
    parameter_table_t parameter_table_;
    parameter_lookup_t parameter_lookup_;
    
    // SVD cutoff value for pseudo inverse, exists in all tasks
    // because Controller implementations need it.
    double sigma_threshold_;
  };
  
}

#endif // OPSPACE_TASK_HPP
