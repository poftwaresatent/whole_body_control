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

#ifndef OPSPACE_PARSE_YAML_HPP
#define OPSPACE_PARSE_YAML_HPP

#include <yaml-cpp/yaml.h>
#include <jspace/wrap_eigen.hpp>

namespace opspace {

  using jspace::Vector;
  class Task;
  class Parameter;
  class TaskFactory;
  class Behavior;
  
  
  struct task_parser_s {
    explicit task_parser_s(std::ostream * optional_dbg_os = 0);
    
    std::string type;
    std::string name;
    
    std::vector<Parameter *> parsed_params;
    
    /** After successfully parsing a YAML node, this contains the
	pointer to the freshly created task. If something goes wrong,
	task will be zero.
	
	\note You are responsible for eventually deleting this Task
	instance.
    */
    Task * task;
    
    /** Set this to a non-zero value to receive debug output from the
	parser. */
    std::ostream * dbg;
  };
  
  
  struct behavior_parser_s {
    behavior_parser_s(TaskFactory const & tfac, std::ostream * optional_dbg_os = 0);
    
    TaskFactory const & tfac;
    std::string type;
    std::string name;
    
    Behavior * behavior;

    std::ostream * dbg;
  };
  
  
  void operator >> (YAML::Node const & node, Vector & vector);
  
  void operator >> (YAML::Node const & node, task_parser_s & task);
  
  void operator >> (YAML::Node const & node, behavior_parser_s & behavior);
  
}

#endif // OPSPACE_PARSE_YAML_HPP
