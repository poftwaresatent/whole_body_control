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
#include <opspace/Parameter.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace opspace {
  
  
  typedef enum {
    TASK_SLOT_DEFAULT = 0,
    TASK_SLOT_OPTIONAL = 1
  } task_slot_flags_t;
  
  
  class TaskSlotAPI
  {
  public:
    std::string const name_;
    task_slot_flags_t const flags_;
    
    TaskSlotAPI(std::string const & name, task_slot_flags_t flags)
      : name_(name), flags_(flags) {}
    
    virtual ~TaskSlotAPI() {}
    
    virtual Status assign(boost::shared_ptr<Task> instance)
    { return Status(false, "type mismatch"); }
    
    virtual size_t getNInstances() const = 0;
    virtual boost::shared_ptr<Task> getInstance(size_t index) = 0;
    
    inline bool isOptional() const { return flags_ & TASK_SLOT_OPTIONAL; }
  };
  
  
  template<typename task_subtype>
  class TaskSlot
    : public TaskSlotAPI
  {
  public:
    TaskSlot(std::string const & name,
	     task_subtype ** slot,
	     task_slot_flags_t flags)
      : TaskSlotAPI(name, flags), slot_(slot) {}
    
    virtual Status assign(boost::shared_ptr<Task> instance) {
      Task * base(instance.get());
      if ( ! base) {
	return Status(false, "null instance");
      }
      task_subtype * raw(dynamic_cast<task_subtype *>(base));
      if ( ! raw) {
	return Status(false, "type mismatch");
      }
      if (slot_) {
	*slot_ = raw;
      }
      instances_.push_back(instance);
      return Status();
    }

    virtual size_t getNInstances() const { return instances_.size(); }
    virtual boost::shared_ptr<Task> getInstance(size_t index) { return instances_[index]; }
    
  protected:
    task_subtype ** slot_;
    std::vector<boost::shared_ptr<Task> > instances_;
  };
  
  
  class Behavior
    : public ParameterReflection
  {
  public:
    typedef std::vector<Task *> task_table_t;
    
    virtual ~Behavior();
    
    virtual Status update(Model const & model) = 0;
    virtual task_table_t const * getTaskTable() = 0;
    
    virtual Status init(Model const & model);
    virtual Status checkJStarSV(Task const * task, Vector const & sv) { Status ok; return ok; }
    
    inline std::string const & getName() const { return name_; }
    
    boost::shared_ptr<TaskSlotAPI> lookupSlot(std::string const & name);
    
    virtual void dump(std::ostream & os,
		      std::string const & title,
		      std::string const & prefix) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    Behavior(std::string const & name);
    
    template<typename task_subtype>
    boost::shared_ptr<TaskSlotAPI>
    declareSlot(std::string const & name,
		task_subtype ** slot = 0,
		task_slot_flags_t flags = TASK_SLOT_DEFAULT)
    {
      boost::shared_ptr<TaskSlotAPI>
	slot_api(new TaskSlot<task_subtype>(name, slot, flags));
      slot_map_[name] = slot_api;
      return slot_api;
    }
    
    std::string const name_;
    
  private:
    typedef std::map<std::string, boost::shared_ptr<TaskSlotAPI> > slot_map_t;
    slot_map_t slot_map_;
  };
  
}

#endif // OPSPACE_BEHAVIOR_HPP
