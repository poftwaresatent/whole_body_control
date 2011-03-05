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
#include <opspace/Factory.hpp>
#include <opspace/Task.hpp>
#include <opspace/Behavior.hpp>
#include <stdexcept>

using jspace::pretty_print;
using boost::shared_ptr;


namespace opspace {
  
  
  Parser::
  Parser(Factory const & factory_, std::ostream * optional_dbg_os)
    : factory(factory_),
      dbg(optional_dbg_os)
  {
  }
  
  
  Parser::
  ~Parser()
  {
  }
  
  
  TaskParser::
  TaskParser(Factory const & factory, std::ostream * optional_dbg_os)
    : Parser(factory, optional_dbg_os),
      type("void"),
      name(""),
      task(0)
  {
  }
  
  
  BehaviorParser::
  BehaviorParser(Factory const & factory, std::ostream * optional_dbg_os)
    : Parser(factory, optional_dbg_os),
      type("void"),
      name(""),
      behavior(0)
  {
  }
  
  
  TaskTableParser::
  TaskTableParser(Factory const & factory,
		  Factory::task_table_t & task_table_,
		  std::ostream * optional_dbg_os)
    : Parser(factory, optional_dbg_os),
      task_table(task_table_)
  {
  }
  
  
  BehaviorTableParser::
  BehaviorTableParser(Factory const & factory,
		      Factory::behavior_table_t & behavior_table_,
		      std::ostream * optional_dbg_os)
    : Parser(factory, optional_dbg_os),
      behavior_table(behavior_table_)
  {
  }
  
  
  static char const * yaml_type_name(YAML::Node const & node)
  {
    switch (node.GetType()) {
    case YAML::CT_NONE:
      return "NONE";
    case YAML::CT_SCALAR:
      return "SCALAR";
    case YAML::CT_SEQUENCE:
      return "SEQUENCE";
    case YAML::CT_MAP:
      return "MAP";
    }
    return "unknown";
  }
  
  
  void operator >> (YAML::Node const & node, Vector & vector)
  {
    vector.resize(node.size());
    for (size_t ii(0); ii < node.size(); ++ii) {
      node[ii] >> vector.coeffRef(ii);
    }
  }
  
  
  void operator >> (YAML::Node const & node, TaskParser & parser)
  {
    parser.task = 0;		// in case type or name is undefined
    if (parser.dbg) {
      *parser.dbg << "DEBUG opspace::operator>>(YAML::Node &, task_parser_s &)\n"
		  << "  reading type and name\n";
    }
    node["type"] >> parser.type;
    node["name"] >> parser.name;

    if (parser.dbg) {
      *parser.dbg << "  type = " << parser.type << "  name = " << parser.name << "\n";
    }
    parser.task = createTask(parser.type, parser.name);
    if ( ! parser.task) {
      throw std::runtime_error("createTask(`" + parser.type + "', `" + parser.name + "') failed");
    }
    
    if (parser.dbg) {
      *parser.dbg << "  created task `" << parser.name << "' of type " << parser.type << "\n"
		  << "    parsing parameters:\n";
    }
    for (YAML::Iterator it(node.begin()); it != node.end(); ++it) {
      std::string key;
      it.first() >> key;
      if (("type" == key) || ("name" == key)) {
	continue;
      }
      YAML::Node const & value(it.second());
      
      if (parser.dbg) {
	*parser.dbg << "    trying `" << key << "' (YAML type " << yaml_type_name(value) << ")\n";
      }
      
      Parameter * param(parser.task->lookupParameter(key));
      if ( ! param) {
	throw std::runtime_error("no parameter called `" + key + "' in task '" + parser.name + "`");
      }
      else {
	
	if (PARAMETER_TYPE_INTEGER == param->type_) {
	  if (YAML::CT_SCALAR != value.GetType()) {
	    throw std::runtime_error("parameter `" + key + "' of task '" + parser.name
				     + "` should be scalar (integer)");
	  }
	  int integer;
	  value >> integer;
	  if (parser.dbg) {
	    *parser.dbg << "    setting value " << integer << "\n";
	  }
	  Status const st(param->set(integer));
	  if ( ! st) {
	    throw std::runtime_error("setting parameter `" + key + "' of task '" + parser.name
				     + "` failed: " + st.errstr);
	  }
	}
	
	else if (PARAMETER_TYPE_REAL == param->type_) {
	  if (YAML::CT_SCALAR != value.GetType()) {
	    throw std::runtime_error("parameter `" + key + "' of task '" + parser.name
				     + "` should be scalar (real)");
	  }
	  double real;
	  value >> real;
	  if (parser.dbg) {
	    *parser.dbg << "    setting value " << real << "\n";
	  }
	  Status const st(param->set(real));
	  if ( ! st) {
	    throw std::runtime_error("setting parameter `" + key + "' of task '" + parser.name
				     + "` failed: " + st.errstr);
	  }
	}
	
	else if (PARAMETER_TYPE_VECTOR == param->type_) {
	  if (YAML::CT_SEQUENCE != value.GetType()) {
	    throw std::runtime_error("parameter `" + key + "' of task '" + parser.name
				     + "` should be sequence (vector)");
	  }
	  Vector vector;
	  value >> vector;
	  if (parser.dbg) {
	    pretty_print(vector, *parser.dbg, "    setting vector", "      ");
	  }
	  Status const st(param->set(vector));
	  if ( ! st) {
	    throw std::runtime_error("setting parameter `" + key + "' of task '" + parser.name
				     + "` failed: " + st.errstr);
	  }
	}
	
	else if (PARAMETER_TYPE_MATRIX == param->type_) {
	  throw std::runtime_error("setting parameter `" + key + "' of task '" + parser.name
				   + "` requires MATRIX type which is not (yet) supported");
	}
	
	else {
	  throw std::runtime_error("setting parameter `" + key + "' of task '" + parser.name
				   + "` invalid or VOID type cannot be set");
	}
	
	if (parser.dbg) {
	  param->dump(*parser.dbg, "    ");
	}
	
      }	// end "if (param)"
      
    } // end for (YAML::Iterator ... )
  }
  
  
  void operator >> (YAML::Node const & node, BehaviorParser & parser)
  {
    parser.behavior = 0;		// in case type or name is undefined
    if (parser.dbg) {
      *parser.dbg << "DEBUG opspace::operator>>(YAML::Node &, behavior_parser_s &)\n"
		  << "  reading type and name\n";
    }
    node["type"] >> parser.type;
    node["name"] >> parser.name;

    if (parser.dbg) {
      *parser.dbg << "  type = " << parser.type << "  name = " << parser.name << "\n";
    }
    parser.behavior = createBehavior(parser.type, parser.name);
    if ( ! parser.behavior) {
      throw std::runtime_error("createBehavior(`" + parser.type + "', `" + parser.name + "') failed");
    }
    
    if (parser.dbg) {
      *parser.dbg << "  created behavior `" << parser.name << "' of type " << parser.type << "\n"
		  << "    parsing parameters and tasks:\n";
    }
    for (YAML::Iterator it(node.begin()); it != node.end(); ++it) {
      std::string key;
      it.first() >> key;
      if (("type" == key) || ("name" == key)) {
	continue;
      }
      
      YAML::Node const & slots(it.second());
      if (parser.dbg) {
	*parser.dbg << "    trying `" << key << "' (YAML type " << yaml_type_name(slots) << ")\n";
      }
      
      if (YAML::CT_MAP != slots.GetType()) {
	throw std::runtime_error("entry for `" + key + "' is not a map");
      }
      
      for (YAML::Iterator slot_it(slots.begin()); slot_it != slots.end(); ++slot_it) {
	std::string slot_name;
	slot_it.first() >> slot_name;
	std::string task_name;
	slot_it.second() >> task_name;
	
	shared_ptr<TaskSlotAPI> slot(parser.behavior->lookupSlot(key, slot_name));
	if ( ! slot) {
	  throw std::runtime_error("behavior `" + parser.name + "' has no slot `" + key + "' / `"
				   + slot_name + "'");
	}
	
	shared_ptr<Task> task(parser.factory.findTask(task_name));
	if ( ! task) {
	  throw std::runtime_error("no task instance `" + task_name + "' for behavior `" + parser.name
				   + "' slot `" + key + "' / `" + slot_name + "'");
	}
	
	Status const st(slot->assign(task));
	if ( ! st) {
	  throw std::runtime_error("oops assigning task instance `" + task_name + "' to behavior `" + parser.name
				   + "' slot `" + key + "' / `" + slot_name + "': " + st.errstr);
	}
	
	if (parser.dbg) {
	  *parser.dbg << "  assigned task instance `" << task_name << "' to slot `" << slot_name
		      << "' of state `" << key << "' in behavior `" << parser.name << "'\n";
	}
      }
    }
  }
  
}
