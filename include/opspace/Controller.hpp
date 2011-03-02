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

#ifndef OPSPACE_CONTROLLER_HPP
#define OPSPACE_CONTROLLER_HPP

#include <opspace/Task.hpp>
#include <boost/shared_ptr.hpp>

namespace opspace {


  /**
     Semi-abstract base class for operational space controllers. This
     base class handles the registration of tasks, and subclasses have
     to provide the computeCommand() method. Optionally, they can also
     override init().
  */
  class Controller
  {
  public:
    typedef std::vector<boost::shared_ptr<Task> > task_table_t;
    
    explicit Controller(std::string const & name, std::ostream * dbg = 0);
    virtual ~Controller() {}
    
    std::string const & getName() const { return name_; }
    
    /**
       Append a task instance to the hierarchy managed by this
       controller.
       
       \return True on success (false if we have already been
       initialized).
    */
    bool appendTask(boost::shared_ptr<Task> task);
    
    task_table_t const & getTaskTable() const { return task_table_; };
    
    /**
       \todo Refactor into full-fledged behavior setup.

       \return true on success (false if we are already initialized)
    */
    bool setFallbackTask(boost::shared_ptr<Task> task);
    
    /**
       The default implementation simply loops over the task table and
       calls Task::init() on all of them, returning an error if one of
       them fails to init. On success, it also sets the initialized_
       flag to true, so that subclasses can use that information if
       desired.
    */
    virtual Status init(Model const & model);
    
    /**
       Abstract method that has to be implemented in order to update
       all tasks and turn their commands into desired joint torques.
     */
    virtual Status computeCommand(Model const & model, Vector & gamma) = 0;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const {}
    
  protected:
    std::string const name_;
    std::ostream * dbg_;
    task_table_t task_table_;
    bool initialized_;
    boost::shared_ptr<Task> fallback_task_;
  };
  
  
  /**
     "The" reference controller based on original code that Luis
     developed during his thesis. Performs dynamically consistent
     nullspace projection magic.
  */
  class LController
    : public Controller
  {
  public:
    explicit LController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status init(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & gamma);
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    std::vector<Vector> sv_lstar_;
    std::vector<Vector> sv_jstar_;
    bool fallback_;
  };
  
}

#endif // OPSPACE_CONTROLLER_HPP
