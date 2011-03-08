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

#include <jspace/Controller.hpp>

class taoDNode;

namespace opspace {
  
  using jspace::Vector;
  using jspace::Model;
  using jspace::Status;
  
  class TypeIOTGCursor;
  struct task_posture_info_getter_s;
  

  /**
     Temporary wrapper class for trying out task/posture control using
     the jspace::Controller API.
     
     \note This class will be discarded as soon as we're happy with
     the opspace::Task and opspace::Controller redesign... (real soon
     nowm seriously).
   */  
  class TaskPostureController
    : public jspace::Controller
  {
  public:
    TaskPostureController();
    virtual ~TaskPostureController();
    
    //////////////////////////////////////////////////
    // implement and override jspace::Controller
    
    virtual jspace::controller_info_getter_s const * getInfo() const;
    virtual Status init(Model const & model);
    virtual Status setGoal(Vector const & goal);
    virtual Status getGoal(Vector & goal) const;
    virtual Status getActual(Vector & actual) const;
    virtual Status setGains(Vector const & kp, Vector const & kd);
    virtual Status getGains(Vector & kp, Vector & kd) const;
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & tau);
    
    //////////////////////////////////////////////////
    // additional functionality (the TaskParameter interface for
    // opspace::Task instances will make this kind of thing way more
    // flexible and reusable)
    
    virtual Status setCycleTime(double dt_seconds);
    virtual Status setEndEffector(taoDNode const * end_effector);
    virtual Status setControlPoint(Vector const & control_point);
    virtual Status getControlPoint(Vector & control_point) const;
    virtual Status setMaxvel(Vector const & maxvel);
    virtual Status getMaxvel(Vector & maxvel) const;
    virtual Status setMaxacc(Vector const & maxacc);
    virtual Status getMaxacc(Vector & maxacc) const;
    
  protected:
    struct level_s {
      level_s();
      ~level_s();
      
      TypeIOTGCursor * cursor;
      Vector goal;
      Vector maxvel;
      Vector maxacc;
      Vector kp;
      Vector kd;
      bool goal_changed;
    };
    
    mutable task_posture_info_getter_s * info_getter_;
    
    double dt_seconds_;
    taoDNode const * end_effector_;
    mutable Vector control_point_;
    Vector actual_;
    level_s task_;
    level_s posture_;
    
    Status updateActual(Model const & model);
  };
  
}

#endif // OPSPACE_CONTROLLER_LIBRARY_HPP
