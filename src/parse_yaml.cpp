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

#include <opspace/parse_yaml.hpp>
#include <opspace/TaskFactory.hpp>
#include <opspace/Task.hpp>
#include <opspace/Behavior.hpp>
#include <stdexcept>

using jspace::pretty_print;
using boost::shared_ptr;


namespace opspace {
  
  
  task_parser_s::
  task_parser_s(std::ostream * optional_dbg_os)
    : type("void"),
      name(""),
      task(0),
      dbg(optional_dbg_os)
  {
  }
  
  
  behavior_parser_s::
  behavior_parser_s(TaskFactory const & tfac_, std::ostream * optional_dbg_os)
    : tfac(tfac_),
      type("void"),
      name(""),
      behavior(0),
      dbg(optional_dbg_os)
  {
  }
  
  
  void operator >> (YAML::Node const & node, Vector & vector)
  {
    vector.resize(node.size());
    for (size_t ii(0); ii < node.size(); ++ii) {
      node[ii] >> vector.coeffRef(ii);
    }
  }
  
  
  void operator >> (YAML::Node const & node, task_parser_s & task)
  {
    task.task = 0;		// in case type or name is undefined
    task.parsed_params.clear();
    if (task.dbg) {
      *task.dbg << "DEBUG opspace::operator>>(YAML::Node &, task_parser_s &)\n"
		<< "  reading type and name\n";
    }
    node["type"] >> task.type;
    node["name"] >> task.name;

    if (task.dbg) {
      *task.dbg << "  type = " << task.type << "  name = " << task.name << "\n";
    }
    task.task = createTask(task.type, task.name);
    if ( ! task.task) {
      throw std::runtime_error("createTask(`" + task.type + "', `" + task.name + "') failed");
    }
    
    if (task.dbg) {
      *task.dbg << "  created task `" << task.name << "' of type " << task.type << "\n"
		<< "    parsing parameters:\n";
    }
    for (YAML::Iterator it(node.begin()); it != node.end(); ++it) {
      std::string key;
      it.first() >> key;
      if (("type" == key) || ("name" == key)) {
	continue;
      }
      YAML::Node const & value(it.second());
      
      if (task.dbg) {
	char const * dbg_type(0);
	switch (value.GetType()) {
	case YAML::CT_NONE:     dbg_type = "NONE"; break;
	case YAML::CT_SCALAR:   dbg_type = "SCALAR"; break;
	case YAML::CT_SEQUENCE: dbg_type = "SEQUENCE"; break;
	case YAML::CT_MAP:      dbg_type = "MAP"; break;
	default: dbg_type = "unknown"; break;
	}
	*task.dbg << "    trying `" << key << "' with YAML type " << dbg_type << "\n";
      }
      
      Parameter * param(task.task->lookupParameter(key));
      if ( ! param) {
	throw std::runtime_error("no parameter called `" + key + "' in task '" + task.name + "`");
      }
      else {
	
	if (TASK_PARAM_TYPE_INTEGER == param->type_) {
	  if (YAML::CT_SCALAR != value.GetType()) {
	    throw std::runtime_error("parameter `" + key + "' of task '" + task.name
				     + "` should be scalar (integer)");
	  }
	  int integer;
	  value >> integer;
	  if (task.dbg) {
	    *task.dbg << "    setting value " << integer << "\n";
	  }
	  Status const st(param->set(integer));
	  if ( ! st) {
	    throw std::runtime_error("setting parameter `" + key + "' of task '" + task.name
				     + "` failed: " + st.errstr);
	  }
	}
	
	else if (TASK_PARAM_TYPE_REAL == param->type_) {
	  if (YAML::CT_SCALAR != value.GetType()) {
	    throw std::runtime_error("parameter `" + key + "' of task '" + task.name
				     + "` should be scalar (real)");
	  }
	  double real;
	  value >> real;
	  if (task.dbg) {
	    *task.dbg << "    setting value " << real << "\n";
	  }
	  Status const st(param->set(real));
	  if ( ! st) {
	    throw std::runtime_error("setting parameter `" + key + "' of task '" + task.name
				     + "` failed: " + st.errstr);
	  }
	}
	
	else if (TASK_PARAM_TYPE_VECTOR == param->type_) {
	  if (YAML::CT_SEQUENCE != value.GetType()) {
	    throw std::runtime_error("parameter `" + key + "' of task '" + task.name
				     + "` should be sequence (vector)");
	  }
	  Vector vector;
	  value >> vector;
	  if (task.dbg) {
	    pretty_print(vector, *task.dbg, "    setting vector", "      ");
	  }
	  Status const st(param->set(vector));
	  if ( ! st) {
	    throw std::runtime_error("setting parameter `" + key + "' of task '" + task.name
				     + "` failed: " + st.errstr);
	  }
	}
	
	else if (TASK_PARAM_TYPE_MATRIX == param->type_) {
	  throw std::runtime_error("setting parameter `" + key + "' of task '" + task.name
				   + "` requires MATRIX type which is not (yet) supported");
	}
	
	else {
	  throw std::runtime_error("setting parameter `" + key + "' of task '" + task.name
				   + "` invalid or VOID type cannot be set");
	}
	
	if (task.dbg) {
	  param->dump(*task.dbg, "    ");
	}
	task.parsed_params.push_back(param);
	
      }	// end "if (param)"
      
    } // end for (YAML::Iterator ... )
  }
  
  
  void operator >> (YAML::Node const & node, behavior_parser_s & behavior)
  {
    behavior.behavior = 0;		// in case type or name is undefined
    if (behavior.dbg) {
      *behavior.dbg << "DEBUG opspace::operator>>(YAML::Node &, behavior_parser_s &)\n"
		<< "  reading type and name\n";
    }
    node["type"] >> behavior.type;
    node["name"] >> behavior.name;

    if (behavior.dbg) {
      *behavior.dbg << "  type = " << behavior.type << "  name = " << behavior.name << "\n";
    }
    behavior.behavior = createBehavior(behavior.type, behavior.name);
    if ( ! behavior.behavior) {
      throw std::runtime_error("createBehavior(`" + behavior.type + "', `" + behavior.name + "') failed");
    }
    
    if (behavior.dbg) {
      *behavior.dbg << "  created behavior `" << behavior.name << "' of type " << behavior.type << "\n"
		    << "    parsing tasks:\n";
    }
    for (YAML::Iterator it(node.begin()); it != node.end(); ++it) {
      std::string key;
      it.first() >> key;
      if (("type" == key) || ("name" == key)) {
	continue;
      }
      
      YAML::Node const & slots(it.second());
      
      if (YAML::CT_MAP != slots.GetType()) {
	throw std::runtime_error("entry for `" + key + "' is not a map");
      }
      
      for (YAML::Iterator slot_it(slots.begin()); slot_it != slots.end(); ++slot_it) {
	std::string slot_name;
	slot_it.first() >> slot_name;
	std::string task_name;
	slot_it.second() >> task_name;
	
	shared_ptr<TaskSlotAPI> slot(behavior.behavior->lookupSlot(key, slot_name));
	if ( ! slot) {
	  throw std::runtime_error("behavior `" + behavior.name + "' has no slot `" + key + "' / `"
				   + slot_name + "'");
	}
	
	shared_ptr<Task> task(behavior.tfac.findTask(task_name));
	if ( ! task) {
	  throw std::runtime_error("no task instance `" + task_name + "' for behavior `" + behavior.name
				   + "' slot `" + key + "' / `" + slot_name + "'");
	}
	
	Status const st(slot->assign(task));
	if ( ! st) {
	  throw std::runtime_error("oops assigning task instance `" + task_name + "' to behavior `" + behavior.name
				   + "' slot `" + key + "' / `" + slot_name + "': " + st.errstr);
	}
	
	if (behavior.dbg) {
	  *behavior.dbg << "  assigned task instance `" << task_name << "' to slot `" << slot_name
			<< "' of state `" << key << "' in behavior `" << behavior.name << "'\n";
	}
      }
    }
  }
  
}
