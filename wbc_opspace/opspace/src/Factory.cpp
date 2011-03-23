/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen
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

#include <opspace/Factory.hpp>
#include <opspace/Behavior.hpp>
#include <opspace/task_library.hpp>
#include <opspace/behavior_library.hpp>
#include <opspace/parse_yaml.hpp>
#include <fstream>
#include <stdexcept>

using jspace::pretty_print;

namespace opspace {
  
  
  /**
     \todo Make this e.g. a non-static member function and use a
     dictionary of subclass creators in order to support adding
     name-type mappings at runtime.
  */
  Task * createTask(std::string const & type, std::string const & name)
  {
    if ("opspace::SelectedJointPostureTask" == type) {
      return new opspace::SelectedJointPostureTask(name);
    }
    if ("opspace::CartPosTrjTask" == type) {
      return new opspace::CartPosTrjTask(name);
    }
    if ("opspace::JPosTrjTask" == type) {
      return new opspace::JPosTrjTask(name);
    }
    if ("opspace::CartPosTask" == type) {
      return new opspace::CartPosTask(name);
    }
    if ("opspace::JPosTask" == type) {
      return new opspace::JPosTask(name);
    }
    if ("opspace::JointLimitTask" == type) {
      return new opspace::JointLimitTask(name);
    }
    if ("opspace::OrientationTask" == type) {
      return new opspace::OrientationTask(name);
    }
    return 0;
  }
  
  
  Status Factory::
  parseString(std::string const & yaml_string)
  {
    std::istringstream is(yaml_string);
    return parseStream(is);
  }
  
  
  Status Factory::
  parseFile(std::string const & yaml_filename)
  {
    std::ifstream is(yaml_filename.c_str());
    if ( ! is) {
      return Status(false, "could not open file `" + yaml_filename + "' for reading");
    }
    return parseStream(is);
  }


  Status Factory::
  parseStream(std::istream & yaml_istream)
  {
    Status st;
    boost::shared_ptr<Task> task;
    
    try {
      YAML::Parser parser(yaml_istream);
      YAML::Node doc;
      TaskTableParser task_table_parser(*this, task_table_, dbg_);
      BehaviorTableParser behavior_table_parser(*this, behavior_table_, dbg_);
      
      parser.GetNextDocument(doc); // <sigh>this'll have merge conflicts again</sigh>
      //while (parser.GetNextDocument(doc)) {
      {
	for (YAML::Iterator ilist(doc.begin()); ilist != doc.end(); ++ilist) {
	  for (YAML::Iterator idict(ilist->begin()); idict != ilist->end(); ++idict) {
	    std::string key;
	    idict.first() >> key;
	    if ("tasks" == key) {
	      idict.second() >> task_table_parser;
	    }
	    else if ("behaviors" == key) {
	      idict.second() >> behavior_table_parser;
	    }
	    else {
	      throw std::runtime_error("invalid key `" + key + "'");
	    }
	  }
	}
      }
    }
    catch (YAML::Exception const & ee) {
      if (dbg_) {
	*dbg_ << "YAML::Exception: " << ee.what() << "\n";
      }
      st.ok = false;
      st.errstr = ee.what();
    }
    catch (std::runtime_error const & ee) {
      if (dbg_) {
	*dbg_ << "std::runtime_error: " << ee.what() << "\n";
      }
      st.ok = false;
      st.errstr = ee.what();
    }
    
    return st;
  }
  
  
  Factory::task_table_t const & Factory::
  getTaskTable() const
  {
    return task_table_;
  }
  
  
  Factory::behavior_table_t const & Factory::
  getBehaviorTable() const
  {
    return behavior_table_;
  }
  
  
  void Factory::
  dump(std::ostream & os,
       std::string const & title,
       std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "  tasks:\n";
    for (task_table_t::const_iterator it(task_table_.begin());
	 it != task_table_.end(); ++it) {
      (*it)->dump(os, "", prefix + "    ");
    }
    os << prefix << "  behaviors:\n";
    for (behavior_table_t::const_iterator it(behavior_table_.begin());
	 it != behavior_table_.end(); ++it) {
      (*it)->dump(os, "", prefix + "    ");
    }
  }

  
  boost::shared_ptr<Task> Factory::
  findTask(std::string const & name)
    const
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      if (name == task_table_[ii]->getName()) {
	return task_table_[ii];
      }
    }
    return boost::shared_ptr<Task>();
  }

  
  boost::shared_ptr<Behavior> Factory::
  findBehavior(std::string const & name)
    const
  {
    for (size_t ii(0); ii < behavior_table_.size(); ++ii) {
      if (name == behavior_table_[ii]->getName()) {
	return behavior_table_[ii];
      }
    }
    return boost::shared_ptr<Behavior>();
  }
  
  
  Behavior * createBehavior(std::string const & type, std::string const & name)
  {
    if ("opspace::TaskPostureBehavior" == type) {
      return new opspace::TaskPostureBehavior(name);
    }
    if ("opspace::TaskPostureTrjBehavior" == type) {
      return new opspace::TaskPostureTrjBehavior(name);
    }
    if ("opspace::HelloGoodbyeBehavior" == type) {
      return new opspace::HelloGoodbyeBehavior(name);
    }
    return 0;
  }
  
  

}
