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

#ifndef OPSPACE_FACTORY_HPP
#define OPSPACE_FACTORY_HPP

#include <jspace/Status.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>


namespace opspace {
  
  using jspace::Status;
  
  class Task;
  class Skill;
  class ReflectionRegistry;
  
  /**
     \todo Replace this with some sort of plugin-based approach.
  */
  Task * createTask(std::string const & type, std::string const & name);
  
  Skill * createSkill(std::string const & type, std::string const & name);
  
  
  /**
     Utility for creating Task and Skill instances based on type
     names. For the moment, we use a hardcoded mapping from type names
     to subclasses, but it will be really easy to extend this with
     some sort of plugin approach.
     
     The idea is to call the parseFoo() methods to feed it with task
     and skill specifications, and then retrieve them using
     getTaskTable() and/or getSkillTable().
     
     The YAML format is quite simple: you specify a list of
     dictionaries. When the key is `tasks', then it parses a list of
     tasks. Similarly, it parses skills when the dictionary key is
     `skills'. You can mix and match, but beware that tasks
     referenced by a skill must be defined before they can be used.
     
     For tasks, the specification is again a list of
     dictionaries. Each one specifies a task. The required dictionary
     keys are `type' and `name'. All the others get looked up via
     Task::lookupParameter() and directly specify task parameters.
     
     Skills are very similar, except that dictionary values are
     handled differently depending on whether they are lists or
     dictionaries: if they are dictionaries, they are treated as state
     definitions which define which task goes where in the slots
     provided by the skill. Otherwise, they are treated as
     parameter definitions.
     
     An example task YAML file is:
     \verbatim
     - tasks:
       - type: opspace::PositionTask
         name: eepos
         dt_seconds: 0.002
         kp: [ 100.0 ]
         kd: [  20.0 ]
         maxvel: [ 0.5 ]
         maxacc: [ 1.5 ]
       - type: opspace::PostureTask
         name: posture
         dt_seconds: 0.002
         kp: [ 400.0, 400.0, 400.0, 100.0, 100.0, 100.0, 100.0 ]
         kd: [  40.0,  40.0,  40.0,  20.0,  20.0,  20.0,  20.0 ]
         maxvel: [ 3.1416 ]
         maxacc: [ 6.2832 ]
     - skills:
       - type: opspace::TPSkill
         name: task_posture
	 # dictionaries define task slots
	 default:
	   eepos: eepos
	   posture: posture
	 # key-value pairs define parameters
	 some_param_name: some_param_value
     \endverbatim
  */
  class Factory
  {
  public:
    typedef std::vector<boost::shared_ptr<Task> > task_table_t;
    typedef std::vector<boost::shared_ptr<Skill> > skill_table_t;
    
    /**
       If you pass a non-zero dbg parameter, the Factory will
       print all sorts of debug information to that stream.
    */
    explicit Factory(std::ostream * dbg = 0) : dbg_(dbg) {}
    
    /**
       Parse a YAML document contained in a string. Retrieve the
       result using getTaskTable().
    */
    Status parseString(std::string const & yaml_string);
    
    /**
       Parse a YAML file, specified as a filename. Retrieve the result
       using getTaskTable().
    */
    Status parseFile(std::string const & yaml_filename);
    
    /**
       Parse a YAML file from a stream. Retrieve the result using
       getTaskTable().
    */
    Status parseStream(std::istream & yaml_istream);
    
    /**
       The task table contains pointers to all task instances ever
       created by this Factory, in the order that they were
       encountered in the YAML documents.
    */
    task_table_t const & getTaskTable() const;
    
    /**
       The skill table contains pointers to all skill instances
       ever created by this Factory, in the order that they were
       encountered in the YAML documents.
    */
    skill_table_t const & getSkillTable() const;
    
    boost::shared_ptr<Task> findTask(std::string const & name) const;
    boost::shared_ptr<Skill> findSkill(std::string const & name) const;
    
    /**
       Create a ReflectionRegistry and populate it with the currently
       registered task and skill instances.
    */
    ReflectionRegistry * createRegistry();
    
    /**
       Write a human-readable (hopefully, anyway) description of all
       skills and tasks contained in the tables.
    */
    void dump(std::ostream & os,
	      std::string const & title,
	      std::string const & prefix) const;
    
  protected:
    std::ostream * dbg_;
    task_table_t task_table_;
    skill_table_t skill_table_;
  };
  
}

#endif // OPSPACE_FACTORY_HPP
