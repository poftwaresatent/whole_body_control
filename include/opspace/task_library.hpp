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
  

  /**
     Base class for tasks with proportional-derivative control. It
     provides a reusable implementation for the parameters and
     algorithm of a PD controller with velocity saturation.
     
     Parameters:
     - goalpos (vector): desired position
     - goalvel (vector): desired velocity
     - kp (vector): proportional gain
     - kd (vector): derivative gain
     - maxvel (vector): velocity saturation limit
     
     Subclasses should call initPDTask() from within their init()
     method, and computePDCommand() from their computeCommand().
   */  
  class PDTask
    : public Task
  {
  public:
    /**
       Verifies that kp, kd, and maxvel are non-negative. If
       initialized, also verifies the vector dimension for kp, kd,
       maxvel, goalpos, and goalvel.
       
       \return Failure if any of the mentioned checks fail, and
       success otherwise.
    */
    virtual Status check(Vector const * param, Vector const & value) const;
    
  protected:
    explicit PDTask(std::string const & name);
    
    /**
       Initialize the goalpos to initpos and the goalvel to zero. Also
       performs sanity checks on kp, kd, and maxvel. If you pass
       allow_scalar_to_vector=true, then any one-dimensional parameter
       values get converted to N-dimensional vectors by filling them
       with N copies of the value. E.g. if kp=[100.0] and initpos is
       3-dimensional, kp would end up as [100.0, 100.0, 100.0].
       
       \return Success if everything went well, failure otherwise.
    */
    Status initPDTask(Vector const & initpos,
		      bool allow_scalar_to_vector);
    
    /**
       Compute velocity-saturated PD command. This boils down to
       driving the task to achieving goalpos with goalvel.
       
       Velocity saturation can be component_wise or not. In the former
       case, each component is scaled according to its saturation
       term. In the latter case, the most saturated component
       determines the scaling of the entire vector.
       
       \return Success if everything went well, failure otherwise.
    */
    Status computePDCommand(Vector const & curpos,
			    Vector const & curvel,
			    bool component_wise_saturation,
			    Vector & command);
    
    bool initialized_;
    Vector goalpos_;
    Vector goalvel_;
    Vector kp_;
    Vector kd_;
    Vector maxvel_;
  };
  
  
  /**
     A test task which drives a subset of DOF to zero using
     non-saturated PD control.
     
     Parameters:
     - selection (vector): values > 0.5 switch on the corresponding DOF
     - kp (vector): proportional gain
     - kd (vector): derivative gain
  */
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
  
  
  /**
     Base class for acceleration-bounded trajectory tasks. Uses a PD
     control law to follow a trajectory generated using the
     reflexxes_otg library,
     
     \todo This should be a subclass of PDTask in order to take
     advantage of the velocity saturation provided by
     PDTask::computePDCommand(), and to reduce code duplication for
     parameter handling.
     
     Parameters:
     - dt_seconds (real): iteration timestep, for trajectory generation
     - goal (vector): goal position (trajectory end point)
     - kp (vector): proportional gain
     - kd (vector): derivative gain
     - maxvel (vector): maximum velocity of generated trajectory
     - maxacc (vector): maximum acceleration of generated trajectory
     
     Subclasses should call initTrajectoryTask() from their init(),
     and computeTrajectoryCommand() from their computeCommand().
  */
  class TrajectoryTask
    : public Task
  {
  public:
    virtual ~TrajectoryTask();
    
    /**
       Checks that dt_seconds is positive.
    */
    virtual Status check(double const * param, double value) const;
    
    /**
       If initialized, checks the validity of goal, kp, kd, maxvel,
       maxacc. Also sets goal_changed_=true in case a new (valid) goal
       was specified.
    */
    virtual Status check(Vector const * param, Vector const & value) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    explicit TrajectoryTask(std::string const & name);
    
    /**
       Initializes the trajectory to be at the current position with
       zero velocity. Also does some sanity checking and optional
       conversion of parameters. If you pass
       allow_scalar_to_vector=true, then single-dimensional parameters
       get blown up to the right size. This is convenient e.g. to set
       a uniform kp for all degrees of freedom.
    */
    Status initTrajectoryTask(Vector const & initpos,
			      bool allow_scalar_to_vector);
    
    /**
       Computes the command for following the trajectory. If the goal
       has been changed since the last time this method was called,
       then it re-initializes the trajectory cursor to generate a
       trajectory to the new goal. Otherwise it just advances the
       cursor by dt_seconds and servos to that position and velocity.
       
       \todo (see also PDTask) implement PD velocity saturation
       at maxvel_ (in addition to the velocity-limited trajectory)
    */
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
  
  
  /**
     Cartesian position trajectory task. Servos a control point,
     specified with respect to a given end_effector link, to the goal
     position.
     
     \note This task is always three dimensional.
     
     Parameters (see also TrajectoryTask for inherited parameters):
     - end_effector_id (integer): identifier of the end effector link
     - control_point (vector): reference point wrt end effector frame
  */
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
  
  
  /**
     Joint-space posture trajectory task. Servos the joint position
     towards a desired posture using acceleration-bounded
     trajectories.
     
     Parameters: inherited from TrajectoryTask.
  */
  class PostureTask
    : public TrajectoryTask
  {
  public:
    explicit PostureTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
  };


  class JointLimitTask
    : public Task
  {
  public:
    explicit JointLimitTask(std::string const & name);
    virtual ~JointLimitTask();
    
    virtual Status check(Vector const * param, Vector const & value) const;
    virtual Status check(double const * param, double const & value) const;
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    // parameters
    Vector upper_stop_deg_;
    Vector upper_trigger_deg_;
    Vector lower_stop_deg_;
    Vector lower_trigger_deg_;
    double dt_seconds_;
    Vector maxvel_;
    Vector maxacc_;
    Vector kp_;
    Vector kd_;
    
    // non-parameters
    Vector upper_stop_;
    Vector upper_trigger_;
    Vector lower_stop_;
    Vector lower_trigger_;
    
    std::vector<TypeIOTGCursor *> cursor_;
    Vector goal_;

    void updateState(Model const & model);

  };
  
}

#endif // OPSPACE_TASK_LIBRARY_HPP
