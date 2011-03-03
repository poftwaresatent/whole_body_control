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

#ifndef OPSPACE_BEHAVIOR_HPP
#define OPSPACE_BEHAVIOR_HPP

#include <opspace/Task.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace opspace {
  
  
  class TaskSlotAPI
  {
  public:
    std::string const state_name_;
    std::string const slot_name_;
    
    TaskSlotAPI(std::string const & state_name,
		std::string const & slot_name,
		boost::shared_ptr<Task> & instance)
      : state_name_(state_name), slot_name_(slot_name), instance_(instance) {}
    
    virtual ~TaskSlotAPI() {}
    
    virtual Status assign(boost::shared_ptr<Task> task) { return Status(false, "type mismatch"); }
    
  protected:
    boost::shared_ptr<Task> & instance_;
  };
  
  
  template<typename task_subtype>
  class TaskSlot
    : public TaskSlotAPI
  {
  public:
    TaskSlot(std::string const & state_name, std::string const & task_name, boost::shared_ptr<Task> & instance)
      : TaskSlotAPI(state_name, task_name, instance) {}
    
    virtual Status assign(boost::shared_ptr<Task> task) {
      if (dynamic_cast<task_subtype *>(task.get())) {
	instance_ = task;
	return Status();
      }
      return Status(false, "type mismatch or null task");
    }
  };
  
  
  class Behavior
  {
  public:
    typedef std::vector<boost::shared_ptr<Task> > task_set_t;
    
    virtual ~Behavior();
    
    virtual Status init(Model const & model) = 0;
    virtual Status update(Model const & model) = 0;
    
    inline std::string const & getName() const { return name_; }
    inline task_set_t const * getTaskSet() { return active_task_set_; }
    
    boost::shared_ptr<TaskSlotAPI> lookupSlot(std::string const & state_name, std::string const & task_name);
    
  protected:
    Behavior(std::string const & name);
    
    template<typename task_subtype>
    void declareSlot(std::string const & state_name,
		     std::string const & slot_name,
		     boost::shared_ptr<task_subtype> task)
    {
      boost::shared_ptr<TaskSlotAPI> slot(new TaskSlot<task_subtype>(state_name, slot_name, task));
      state_map_[state_name][slot_name] = slot;
    }
    
    std::string const name_;
    
    /** Has to be set by subclasses in the update() method. */
    task_set_t const * active_task_set_;
    
  private:
    typedef std::map<std::string, boost::shared_ptr<TaskSlotAPI> > task_slot_map_t;
    typedef std::map<std::string, task_slot_map_t> state_map_t;
    state_map_t state_map_;
  };
  
  
  class TPBehavior
    : public Behavior
  {
  public:
    TPBehavior(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
  protected:
    task_set_t task_set_;
  };
  
  
  // class CleanBoardBehavior
  //   : public Behavior
  // {
  // public:
  //   CleanBoardBehavior(std::string const & name)
  //     : Behavior(name),
  //   {
  //     declareTask("approach_board", "eepos", ???);
  //     declareTask("approach_board", "eeori", ???);
  //     declareTask("approach_board", "posture", ???);
  //     declareTask("wipe", "eepos", ???);
  //     declareTask("wipe", "eeori", ???);
  //     declareTask("wipe", "eeforce", ???);
  //     declareTask("wipe", "posture", ???);
  //   }
  
}

#endif // OPSPACE_BEHAVIOR_HPP
