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

#ifndef OPSPACE_TASK_LIBRARY_HPP
#define OPSPACE_TASK_LIBRARY_HPP

#include <opspace/Task.hpp>

namespace opspace {
  
  class TypeIOTGCursor;
  
  
  class PDTask
    : public Task
  {
  public:
    virtual Status check(Vector const * param, Vector const & value) const;
    
  protected:
    explicit PDTask(std::string const & name);
    
    Status initPDTask(Vector const & initpos,
		      bool allow_scalar_to_vector);
    Status computePDCommand(Vector const & curpos,
			    Vector const & curvel,
			    Vector & command);
    
    bool initialized_;
    Vector goalpos_;
    Vector goalvel_;
    Vector kp_;
    Vector kd_;
    Vector maxvel_;
  };
  
  
  class SelectedJointPostureTask
    : public Task
  {
  public:
    explicit SelectedJointPostureTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual Status check(double const * param, double value) const;
    virtual Status check(Vector const * param, Vector const & value) const;
    
  protected:
    Vector selection_;
    double kp_;
    double kd_;
    bool initialized_;
    std::vector<size_t> active_joints_;
  };
  
  
  class TrajectoryTask
    : public Task
  {
  public:
    virtual ~TrajectoryTask();
    
    virtual Status check(double const * param, double value) const;
    virtual Status check(Vector const * param, Vector const & value) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    explicit TrajectoryTask(std::string const & name);
    
    Status initTrajectoryTask(Vector const & initpos,
			      bool allow_scalar_to_vector);
    Status computeTrajectoryCommand(Vector const & curpos,
				    Vector const & curvel,
				    Vector & command);
    
    TypeIOTGCursor * cursor_;
    double dt_seconds_;
    Vector goal_;
    mutable bool goal_changed_;
    Vector kp_;
    Vector kd_;
    Vector maxvel_;
    Vector maxacc_;
  };
  
  
  class PositionTask
    : public TrajectoryTask
  {
  public:
    explicit PositionTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
  protected:
    int end_effector_id_;
    Vector control_point_;
    
    taoDNode const * updateActual(Model const & model);
  };
  
  
  class PostureTask
    : public TrajectoryTask
  {
  public:
    explicit PostureTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
  };
  
}

#endif // OPSPACE_TASK_LIBRARY_HPP
