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

#include <opspace/Parameter.hpp>
#include <jspace/Model.hpp>

namespace opspace {
  
  
  using jspace::Model;
  
  
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
    : public ParameterReflection
  {
  protected:
    explicit Task(std::string const & name);
    
  public:
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
    
    /**
       \todo Move into superclass, anything with parameters should
       have a name: more consistent behavior for enumeration.
    */
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
       SVD cutoff value for pseudo inverse, exists in all tasks
       because Controller implementations need it.
    */
    double getSigmaThreshold() const { return sigma_threshold_; }
    
    virtual void dump(std::ostream & os,
		      std::string const & title,
		      std::string const & prefix) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    std::string const name_;
    Vector actual_;
    Vector command_;
    Matrix jacobian_;
    
    /** Parameter "sigma_threshold", SVD cutoff value for pseudo
	inverse. Exists in all tasks because Controller
	implementations need it. */
    double sigma_threshold_;
  };
  
}

#endif // OPSPACE_TASK_HPP
