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

#ifndef OPSPACE_CONTROLLER_LIBRARY_HPP
#define OPSPACE_CONTROLLER_LIBRARY_HPP

#include <opspace/Controller.hpp>
#include <boost/shared_ptr.hpp>

namespace opspace {
  
  
  class ClassicTaskPostureController
    : public Controller
  {
  public:
    explicit ClassicTaskPostureController(std::string const & name);
    
    virtual Status init(Model const & model);
    
    virtual Status computeCommand(Model const & model,
				  Skill & skill,
				  Vector & gamma);

    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    Vector jpos_;
    Vector jvel_;
    Vector gamma_;
    Vector fstar_;
    Matrix lambda_;
    Matrix jbar_;
    Matrix nullspace_;
  };
  
  
  class ControllerNG
    : public Controller
  {
  public:
    explicit ControllerNG(std::string const & name);
    
    void setFallbackTask(boost::shared_ptr<Task> task);
    
    virtual Status init(Model const & model);

    virtual Status computeCommand(Model const & model,
				  Skill & skill,
				  Vector & gamma);
    
    virtual Status check(std::string const * param, std::string const & value) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
    Status computeFallback(Model const & model,
			   bool init_required,
			   Vector & gamma);
    
    inline Vector const & getCommand() const { return gamma_; }
    
    void qhlog(Skill & skill, long long timestamp);
    
    
  protected:
    boost::shared_ptr<Task> fallback_task_;
    ////    std::vector<Vector> sv_lstar_; // stored only for dbg()
    bool fallback_;
    std::string fallback_reason_;
    
    std::vector<boost::shared_ptr<ParameterLog> > log_;
    int loglen_;		// <= 0 means disabled
    int logsubsample_;
    std::string logprefix_;
    std::vector<Vector> sv_jstar_;
    
    // -1 means off, 0 means init, -2 means maybeWriteLogFiles() will
    // actually write them (this gets set when ==loglen_)
    mutable int logcount_;
    
    // for logging and debugging via Parameter tools, don't bother to
    // implement check() methods because one day real soon now we'll
    // be able to flag parameters as read-only and then the superclass
    // can take care of signaling errors when someone writes to
    // them...
    Vector jpos_;
    Vector jvel_;
    Vector gamma_;
  };

}

#endif // OPSPACE_CONTROLLER_LIBRARY_HPP
