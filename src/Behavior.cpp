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
    : name_(name)
  {
  }
  
  
  Behavior::
  ~Behavior()
  {
  }
  
  
  Status Behavior::
  init(Model const & model)
  {
    bool ok(true);

    {
      std::ostringstream msg;
      msg << "missing task instances:\n";
      for (state_map_t::const_iterator is(state_map_.begin()); is != state_map_.end(); ++is) {
	for (task_slot_map_t::const_iterator it(is->second.begin()); it != is->second.end(); ++it) {
	  if ( ! it->second->getInstance()) {
	    ok = false;
	    msg << "  state `" << is->first << "' task `" << it->first << "'\n";
	  }
	}
      }
      if ( ! ok) {
	return Status(false, msg.str());
      }
    }
    
    {
      std::ostringstream msg;
      msg << "failed task initializations:\n";
      for (state_map_t::const_iterator is(state_map_.begin()); is != state_map_.end(); ++is) {
	for (task_slot_map_t::const_iterator it(is->second.begin()); it != is->second.end(); ++it) {
	  Status const st(it->second->getInstance()->init(model));
	  if ( ! st) {
	    ok = false;
	    msg << "  state `" << is->first << "' task `" << it->first
		<< "': " << st.errstr << "\n";
	  }
	}
      }
      if ( ! ok) {
	return Status(false, msg.str());
      }
    }
    
    return Status();
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
  
  
  void Behavior::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "behavior " << name_ << "\n";
    for (state_map_t::const_iterator is(state_map_.begin()); is != state_map_.end(); ++is) {
      for (task_slot_map_t::const_iterator it(is->second.begin()); it != is->second.end(); ++it) {
	Task const * task(it->second->getInstance().get());
	if (task) {
	  task->dbg(os, "  " + is->first + "/" + it->first, prefix + "    ");
	}
	else {
	  os << prefix << "  " << is->first << "/" << it->first << " is NULL\n";
	}
      }
    }
  }
  
}
