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

#include <opspace/TaskFactory.hpp>
#include <opspace/Behavior.hpp>
#include <opspace/task_library.hpp>
#include <opspace/behavior_library.hpp>
#include <opspace/parse_yaml.hpp>
#include <fstream>
#include <stdexcept>

using jspace::pretty_print;

namespace opspace {
  
  
  /**
     \todo A little awkward to define this here but use it in
     parse_yaml.cpp, which is essentially just called from here. Ah
     well, long live refactoring...
  */
  Task * createTask(std::string const & type, std::string const & name)
  {
    if ("opspace::SelectedJointPostureTask" == type) {
      return new opspace::SelectedJointPostureTask(name);
    }
    if ("opspace::PositionTask" == type) {
      return new opspace::PositionTask(name);
    }
    if ("opspace::PostureTask" == type) {
      return new opspace::PostureTask(name);
    }
    if ("opspace::JointLimitTask" == type) {
      return new opspace::JointLimitTask(name);
    }
    if ("opspace::OrientationTask" == type) {
      return new opspace::OrientationTask(name);
    }
    return 0;
  }
  
  
  Status TaskFactory::
  parseString(std::string const & yaml_string)
  {
    std::istringstream is(yaml_string);
    return parseStream(is);
  }
  
  
  Status TaskFactory::
  parseFile(std::string const & yaml_filename)
  {
    std::ifstream is(yaml_filename.c_str());
    if ( ! is) {
      return Status(false, "could not open file `" + yaml_filename + "' for reading");
    }
    return parseStream(is);
  }
  

  Status TaskFactory::
  parseStream(std::istream & yaml_istream)
  {
    Status st;
    boost::shared_ptr<Task> task;
    
    try {
      YAML::Parser parser(yaml_istream);
      YAML::Node doc;
      task_parser_s task_parser(dbg_);

      while (parser.GetNextDocument(doc)) {
	for (size_t ii(0); ii < doc.size(); ++ii) {
	  YAML::Node const & node(doc[ii]);
	  node >> task_parser;
	  if ( ! task_parser.task) {
	    throw std::runtime_error("oops, task_parser_s::task is zero");
	  }
	  task.reset(task_parser.task);
	  task_table_.push_back(task);
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
  
  
  TaskFactory::task_table_t const & TaskFactory::
  getTaskTable() const
  {
    return task_table_;
  }
  
  
  void TaskFactory::
  dump(std::ostream & os,
       std::string const & title,
       std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    for (task_table_t::const_iterator it(task_table_.begin());
	 it != task_table_.end(); ++it) {
      (*it)->dump(os, "", prefix + "  ");
    }
  }

  
  /**
     \todo Would be nice to use a std::map and also detect duplicate
     names, which should be errors...
   */
  boost::shared_ptr<Task> TaskFactory::
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
  
  
  Behavior * createBehavior(std::string const & type, std::string const & name)
  {
    if ("opspace::TPBehavior" == type) {
      return new opspace::TPBehavior(name);
    }
    if ("opspace::HelloGoodbyeBehavior" == type) {
      return new opspace::HelloGoodbyeBehavior(name);
    }
    return 0;
  }
  
  

}
