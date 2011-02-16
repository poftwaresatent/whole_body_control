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

namespace opspace {


  class Controller
  {
  public:
    struct task_info_s {
      task_info_s(Task * tt, bool co)
	: task(tt), controller_owned(co) {}
      
      Task * task;
      bool controller_owned;
    };
    
    typedef std::vector<task_info_s *> task_table_t;
    
    explicit Controller(std::string const & name, std::ostream * dbg = 0);
    virtual ~Controller();
    
    std::string const & getName() const { return name_; }
    
    /** \note Transfers ownership of the task object if you set
	controller_owned to true. Controller-owned tasks get deleted
	in the Controller destructor. */
    task_info_s const * appendTask(Task * task, bool controller_owned);
    
    task_table_t const & getTaskTable() const { return task_table_; };
    
    virtual Status init(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & gamma) = 0;
    
  protected:
    std::string const name_;
    std::ostream * dbg_;
    task_table_t task_table_;
    bool initialized_;
  };
  
  
  class SController
    : public Controller
  {
  public:
    explicit SController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status computeCommand(Model const & model, Vector & gamma);
  };
  
  
  class LController
    : public Controller
  {
  public:
    explicit LController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status computeCommand(Model const & model, Vector & gamma);
  };
  
  
  class TPController
    : public Controller
  {
  public:
    explicit TPController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status init(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & gamma);
    
  protected:
    Task * task_;
    Task * posture_;
  };
  
}
