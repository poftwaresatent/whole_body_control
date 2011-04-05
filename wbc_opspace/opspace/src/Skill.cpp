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

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  Skill::
  Skill(std::string const & name)
    : ParameterReflection("skill", name)
  {
  }
  
  
  Skill::
  ~Skill()
  {
  }
  
  
  Status Skill::
  init(Model const & model)
  {
    bool ok(true);

    {
      std::ostringstream msg;
      msg << "missing non-optional task instances:\n";
      for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
	if ((0 == is->second->getNInstances())
	    && ( ! is->second->isOptional())) {
	  ok = false;
	  msg << "  slot `" << is->first << "' task `" << is->first << "'\n";
	}
      }
      if ( ! ok) {
	return Status(false, msg.str());
      }
    }
    
    {
      std::ostringstream msg;
      msg << "failed task initializations:\n";
      for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
	for (size_t it(0); it < is->second->getNInstances(); ++it) {
	  shared_ptr<Task> task(is->second->getInstance(it));
	  Status const st(task->init(model));
	  if ( ! st) {
	    ok = false;
	    msg << "  slot `" << is->first << "' task[" << it << "] `" << task->getName()
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
  
  
  boost::shared_ptr<TaskSlotAPI> Skill::
  lookupSlot(std::string const & name)
  {
    shared_ptr<TaskSlotAPI> slot;
    slot_map_t::iterator ism(slot_map_.find(name));
    if (ism != slot_map_.end()) {
      slot = ism->second;
    }
    return slot;
  }
  
  
  void Skill::
  dump(std::ostream & os,
       std::string const & title,
       std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "skill " << instance_name_ << "\n";
    ParameterReflection::dump(os, prefix + "  parameters:", prefix + "    ");
    os << prefix << "  slots:\n";
    for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
      os << prefix << "    " << is->first;
      if (0 == is->second->getNInstances()) {
	os << " (EMPTY)\n";
      }
      else if (1 == is->second->getNInstances()) {
	os << ": " << is->second->getInstance(0)->getName() << "\n";
      }
      else {
	os << ":\n";
	for (size_t it(0); it < is->second->getNInstances(); ++it) {
	  os << prefix << "      [" << it << "]: "
	     << is->second->getInstance(it)->getName() << "\n";
	}
      }
    }
  }
  
  
  void Skill::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "skill " << instance_name_ << "\n";
    ParameterReflection::dump(os, prefix + "  parameters", prefix + "    ");
    for (slot_map_t::const_iterator is(slot_map_.begin()); is != slot_map_.end(); ++is) {
      for (size_t it(0); it < is->second->getNInstances(); ++it) {
	Task const * task(is->second->getInstance(it).get());
	task->dbg(os, "  " + is->first + "/" + task->getName(), prefix + "    ");
      }
    }
  }
  
}
