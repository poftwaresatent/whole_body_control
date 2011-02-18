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

#ifndef OPSPACE_TASK_FACTORY_HPP
#define OPSPACE_TASK_FACTORY_HPP

#include <yaml-cpp/yaml.h>
#include <jspace/Status.hpp>


namespace opspace {
  
  using jspace::Status;
  
  class Task;
  
  /**
     Utility for creating Task instances based on type names. Mostly
     (currently: exclusively) used for parsing YAML files that specify
     tasks. For the moment, we use a hardcoded mapping from type names
     to Task subclasses, but it will be really easy to extend this
     with some sort of plugin approach (we've done it before and have
     code ready to port, but there are more pressing issues elsewhere
     for now).
     
     The idea is to call the parseFoo() methods to feed it with task
     specifications, and then retrieve them using getTaskTable(). The
     latter returns a vector of pointers to Task (subclass) instances,
     ordered simply by the order in which specifications where
     encountered.
     
     \note The TaskFactory never deletes any of the Task instances it
     creates during parsing. You are responsible for doing that.
     
     The YAML format is quite simple, of course: you specify a list of
     dictionaries. Each list item specifies a task. The required
     dictionary keys are `type' and `name'. All the others get looked
     up via Task::lookupParameter() and directly specify task
     parameters.
     
     An example task YAML file is:
     \verbatim
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
       maxacc: [ 6.2832 ]\n;
     \endverbatim
  */
  class TaskFactory
  {
  public:
    typedef std::vector<Task *> task_table_t;
    
    /**
       If you pass a non-zero dbg parameter, the TaskFactory will
       print all sorts of debug information to that stream.
    */
    explicit TaskFactory(std::ostream * dbg = 0) : dbg_(dbg) {}
    
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
       created by this TaskFactory, in the order that they were
       encountered in the YAML documents. The task instances never get
       deleted by TaskFactory.
    */
    task_table_t const & getTaskTable() const;
    
    /**
       Write a human-readable (hopefully, anyway) description of all
       tasks contained in the task table. Ends up calling Task::dump()
       on all instances of the tasl table.
    */
    void dump(std::ostream & os,
	      std::string const & title,
	      std::string const & prefix) const;
    
  protected:
    std::ostream * dbg_;
    task_table_t task_table_;
  };
  
}

#endif // OPSPACE_TASK_FACTORY_HPP
