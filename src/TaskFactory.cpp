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
#include <opspace/task_library.hpp>
#include <fstream>
#include <stdexcept>

using jspace::pretty_print;

namespace opspace {
  
  
  static void operator >> (YAML::Node const & node, Vector & vector)
  {
    vector.resize(node.size());
    for (size_t ii(0); ii < node.size(); ++ii) {
      node[ii] >> vector.coeffRef(ii);
    }
  }


  static Task * createTask(std::string const & type, std::string const & name)
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
    Task * task(0);
    
    try {
      YAML::Parser parser(yaml_istream);
      YAML::Node doc;

      while (parser.GetNextDocument(doc)) {
	////      for (parser.GetNextDocument(doc); parser; parser.GetNextDocument(doc)) {
	for (size_t ii(0); ii < doc.size(); ++ii) {
	  YAML::Node const & node(doc[ii]);
	  std::string type, name;
	  node["type"] >> type;
	  node["name"] >> name;
	  task = createTask(type, name);
	  if ( ! task) {
	    throw std::runtime_error("opspace::TaskFactory::parseStream(): createTask(`"
				     + type + "', `" + name + "') failed");
	  }
	  if (dbg_) {
	    *dbg_ << "created task `" << name << "' of type " << type << "\n"
		  << "  parsing parameters:\n";
	  }
	  for (YAML::Iterator it(node.begin()); it != node.end(); ++it) {
	    std::string key;
	    it.first() >> key;
	    if (("type" == key) || ("name" == key)) {
	      continue;
	    }
	    YAML::Node const & value(it.second());
	    
	    if (dbg_) {
	      char const * dbg_type(0);
	      switch (value.GetType()) {
	      case YAML::CT_NONE:     dbg_type = "NONE"; break;
	      case YAML::CT_SCALAR:   dbg_type = "SCALAR"; break;
	      case YAML::CT_SEQUENCE: dbg_type = "SEQUENCE"; break;
	      case YAML::CT_MAP:      dbg_type = "MAP"; break;
	      default: dbg_type = "unknown"; break;
	      }
	      *dbg_ << "  trying `" << key << "' with YAML type " << dbg_type << "\n";
	    }
	    
	    Parameter * param(task->lookupParameter(key));
	    if (param) {
	      
	      if (TASK_PARAM_TYPE_INTEGER == param->type_) {
		if (dbg_) {
		  *dbg_ << "  found integer `" << key << "'\n";
		}
		if (YAML::CT_SCALAR != value.GetType()) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): parameter `" + key
					   + "' of task '" + name + "` should be scalar (integer)");
		}
		int integer;
		value >> integer;
		st = param->set(integer);
		if ( ! st) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					   + key + "' of task '" + name + "` failed: " + st.errstr);
		}
	      }
	      
	      else if (TASK_PARAM_TYPE_REAL == param->type_) {
		if (YAML::CT_SCALAR != value.GetType()) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): parameter `" + key
					   + "' of task '" + name + "` should be scalar (real)");
		}
		double real;
		if (dbg_) {
		  std::string dbg_real;
		  value >> dbg_real;
		  *dbg_ << "  attempting to parse real value from `" << dbg_real
			<< "' for parameter `" << key << "'\n";
		}
		value >> real;
		if (dbg_) {
		  *dbg_ << "  setting value " << real << "\n";
		}
		st = param->set(real);
		if ( ! st) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					   + key + "' of task '" + name + "` failed: " + st.errstr);
		}
	      }
		
	      else if (TASK_PARAM_TYPE_VECTOR == param->type_) {
		if (dbg_) {
		  *dbg_ << "  found vector `" << key << "'\n";
		}
		if (YAML::CT_SEQUENCE != value.GetType()) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): parameter `" + key
					   + "' of task '" + name + "` should be sequence (vector)");
		}
		Vector vector;
		value >> vector;
		if (dbg_) {
		  pretty_print(vector, *dbg_, "  setting vector", "    ");
		}
		st = param->set(vector);
		if ( ! st) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					   + key + "' of task '" + name + "` failed: " + st.errstr);
		}
	      }
	      
	      else if (TASK_PARAM_TYPE_MATRIX == param->type_) {
		if (dbg_) {
		  *dbg_ << "  found matrix `" << key << "'\n";
		}
		throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					 + key + "' of task '" + name
					 + "` requires MATRIX type which is not (yet) supported");
	      }
	      
	      else {
		if (dbg_) {
		  *dbg_ << "  found void `" << key << "'\n";
		}
		throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					 + key + "' of task '" + name
					 + "` invalid or VOID type cannot be set");
	      }
	      
	      if (dbg_) {
		param->dump(*dbg_, "  ");
	      }
	    }
	    else if (dbg_) {
	      *dbg_ << "  " << key << " not found\n";
	    }
	    
	  }
	  
	  task_table_.push_back(task);
	}
	
      }
    }
    catch (YAML::Exception const & ee) {
      if (dbg_) {
	*dbg_ << "YAML::Exception: " << ee.what() << "\n";
      }
      delete task;
      st.ok = false;
      st.errstr = ee.what();
    }
    catch (std::runtime_error const & ee) {
      if (dbg_) {
	*dbg_ << "std::runtime_error: " << ee.what() << "\n";
      }
      delete task;
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
  
}
