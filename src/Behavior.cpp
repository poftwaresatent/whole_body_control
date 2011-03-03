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

#include <opspace/Behavior.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  Behavior::
  Behavior(std::string const & name)
    : name_(name),
      active_task_set_(0)
  {
  }
  
  
  Behavior::
  ~Behavior()
  {
  }
  
  
  boost::shared_ptr<TaskSlotAPI> Behavior::
  lookupSlot(std::string const & state_name, std::string const & task_name)
  {
    shared_ptr<TaskSlotAPI> slot;
    state_map_t::iterator ism(state_map_.find(state_name));
    if (ism == state_map_.end()) {
      return slot;
    }
    task_slot_map_t::iterator isd(ism->second.find(task_name));
    if (isd != ism->second.end()) {
      slot = isd->second;
    }
    return slot;
  }
  
  
  TPBehavior::
  TPBehavior(std::string const & name)
    : Behavior(name),
      task_set_(2)
  {
    declareSlot("default", "eepos", task_set_[0]);
    declareSlot("default", "posture", task_set_[1]);
  }
  
  
  Status TPBehavior::
  init(Model const & model)
  {
    for (size_t ii(0); ii < task_set_.size(); ++ii) {
      if ( ! task_set_[ii]) {
	return Status(false, "missing slot assignment");
      }
      Status const st(task_set_[ii]->init(model));
      if ( ! st) {
	return st;
      }
    }
    active_task_set_ = &task_set_;
    Status ok;
    return ok;
  }
  
  
  Status TPBehavior::
  update(Model const & model)
  {
    for (size_t ii(0); ii < task_set_.size(); ++ii) {
      Status const st(task_set_[ii]->update(model));
      if ( ! st) {
	return st;
      }
    }
    Status ok;
    return ok;
  }
  
}
