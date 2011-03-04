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

#ifndef OPSPACE_BEHAVIOR_LIBRARY_HPP
#define OPSPACE_BEHAVIOR_LIBRARY_HPP

#include <opspace/Behavior.hpp>
#include <opspace/task_library.hpp>

namespace opspace {
  

  class TPBehavior
    : public Behavior
  {
  public:
    TPBehavior(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
  protected:
    PositionTask * eepos_;
    PostureTask * posture_;
    task_table_t task_table_;
  };


  class HelloGoodbyeBehavior
    : public Behavior
  {
  public:
    HelloGoodbyeBehavior(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:
    enum {
      STATE_START,
      STATE_SHAKE,
      STATE_WAVE_LEFT,
      STATE_WAVE_RIGHT,
      STATE_RETURN
    } state_;
    
    //    OrientationTask * shake_eeori_;
    PositionTask * shake_eepos_;
    PostureTask * shake_posture_;
    task_table_t shake_;
    
    PositionTask * wave_eepos_;
    PostureTask * wave_posture_;
    task_table_t wave_;
    
    Parameter * shake_eepos_goal_;
    Parameter * wave_eepos_goal_;
    
    Vector shake_position_;
    double shake_distance_;
    double shake_distance_threshold_;
    size_t shake_count_;
    size_t shake_count_threshold_;
    
    Vector wave_position_left_;
    Vector wave_position_right_;
    double wave_distance_left_;
    double wave_distance_right_;
    double wave_distance_threshold_;
    size_t wave_count_;
    size_t wave_count_threshold_;
  };
  
}

#endif // OPSPACE_BEHAVIOR_LIBRARY_HPP
