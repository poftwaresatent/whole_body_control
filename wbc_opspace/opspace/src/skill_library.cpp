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

#include <opspace/skill_library.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  GenericSkill::
  GenericSkill(std::string const & name)
    : Skill(name)
  {
    slot_ = declareSlot<Task>("task");
  }
  
  
  Status GenericSkill::
  init(Model const & model)
  {
    Status const st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    for (size_t ii(0); ii < slot_->getNInstances(); ++ii) {
      task_table_.push_back(slot_->getInstance(ii).get());
    }
    return st;
  }
  
  
  Status GenericSkill::
  update(Model const & model)
  {
    Status st;
    if ( task_table_.empty()) {
      st.ok = false;
      st.errstr = "empty task table, did you assign any? did you forget to init()?";
    }
    else {
      for (size_t ii(0); ii < task_table_.size(); ++ii) {
	st = task_table_[ii]->update(model);
	if ( ! st) {
	  return st;
	}
      }
    }
    return st;
  }
  
  
  Skill::task_table_t const * GenericSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  void GenericSkill::
  appendTask(boost::shared_ptr<Task> task)
  {
    slot_->assign(task);
  }
  
  
  TaskPostureSkill::
  TaskPostureSkill(std::string const & name)
    : Skill(name)
  {
    declareSlot("eepos", &eepos_);
    declareSlot("posture", &posture_);
  }
  
  
  Status TaskPostureSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    task_table_.push_back(eepos_);
    task_table_.push_back(posture_);
    return st;
  }
  
  
  Status TaskPostureSkill::
  update(Model const & model)
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Status const st(task_table_[ii]->update(model));
      if ( ! st) {
	return st;
      }
    }
    Status ok;
    return ok;
  }
  
  
  Skill::task_table_t const * TaskPostureSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status TaskPostureSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (task == eepos_) {
      if (sv.rows() != 3) {
	return Status(false, "eepos dimension mismatch");
      }
      if (sv[2] < eepos_->getSigmaThreshold()) {
	return Status(false, "singular eepos");
      }
    }
    Status ok;
    return ok;
  }


  TaskPostureTrjSkill::
  TaskPostureTrjSkill(std::string const & name)
    : Skill(name)
  {
    declareSlot("eepos", &eepos_);
    declareSlot("posture", &posture_);
  }
  
  
  Status TaskPostureTrjSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    task_table_.push_back(eepos_);
    task_table_.push_back(posture_);
    return st;
  }
  
  
  Status TaskPostureTrjSkill::
  update(Model const & model)
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Status const st(task_table_[ii]->update(model));
      if ( ! st) {
	return st;
      }
    }
    Status ok;
    return ok;
  }
  
  
  Skill::task_table_t const * TaskPostureTrjSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status TaskPostureTrjSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (task == eepos_) {
      if (sv.rows() != 3) {
	return Status(false, "eepos dimension mismatch");
      }
      if (sv[2] < eepos_->getSigmaThreshold()) {
	return Status(false, "singular eepos");
      }
    }
    Status ok;
    return ok;
  }
  
  
  HelloGoodbyeSkill::
  HelloGoodbyeSkill(std::string const & name)
    : Skill(name),
      state_(STATE_START),
      //      shake_eeori_(0),
      shake_eepos_task_(0),
      shake_posture_task_(0),
      wave_eepos_task_(0),
      wave_posture_task_(0),
      shake_eepos_goal_(0),
      wave_eepos_goal_(0),
      wave_posture_goal_(0),
      shake_position_(Vector::Zero(3)),
      shake_posture_(Vector::Zero(7)), // XXXX hardcoded for dreamer
      shake_distance_(0.0),
      shake_distance_threshold_(0.08),
      shake_count_(0),
      shake_count_threshold_(700),
      wave_position_left_(Vector::Zero(3)),
      wave_position_right_(Vector::Zero(3)),
      wave_posture_(Vector::Zero(7)), // XXXX hardcoded for dreamer
      wave_distance_left_(0.0),
      wave_distance_right_(0.0),
      wave_distance_threshold_(0.1),
      wave_count_(0),
      wave_count_threshold_(6)
  {
    //    declareSlot("shake", "orientation", &shake_eeori_);
    declareSlot("shake_position", &shake_eepos_task_);
    declareSlot("shake_posture", &shake_posture_task_);
    declareSlot("wave_position", &wave_eepos_task_);
    declareSlot("wave_posture", &wave_posture_task_);
  }
  
  
  Status HelloGoodbyeSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    
    // XXXX to do: could read the name of the parameter from a
    // parameter (also for other tasks)
    shake_eepos_goal_ = shake_eepos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! shake_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in shake position task";
      return st;
    }
    
    shake_posture_goal_ = shake_posture_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! shake_posture_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in shake posture task";
      return st;
    }
    
    wave_eepos_goal_ = wave_eepos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! wave_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in wave position task";
      return st;
    }
    
    wave_posture_goal_ = wave_posture_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! wave_posture_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in wave position task";
      return st;
    }
    
    //    shake_.push_back(shake_eeori_);
    shake_task_table_.push_back(shake_eepos_task_);
    shake_task_table_.push_back(shake_posture_task_);
    wave_task_table_.push_back(wave_eepos_task_);
    wave_task_table_.push_back(wave_posture_task_);
    
    state_ = STATE_START;
    shake_position_ = shake_eepos_task_->getActual();
    shake_posture_ = shake_posture_task_->getActual();
    if (7 != shake_posture_.rows()) {
      st.ok = false;
      st.errstr = "shake posture is not 7 dimensional";
      return st;
    }
    wave_position_left_ <<  0.30, -0.07, 0.17;
    wave_position_right_ << 0.29, -0.33, 0.20;
    wave_posture_ <<        1.24, 0.06, -0.53, 1.87, -0.10, -0.24, 0.14;
    
    return st;
  }
  
  
  Status HelloGoodbyeSkill::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < shake_task_table_.size(); ++ii) {
      st = shake_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    for (size_t ii(0); ii < wave_task_table_.size(); ++ii) {
      st = wave_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    
    Vector const shake_delta(shake_position_ - shake_eepos_task_->getActual());
    shake_distance_ = shake_delta.norm();
    Vector const wave_delta_left(wave_position_left_ - wave_eepos_task_->getActual());
    wave_distance_left_ = wave_delta_left.norm();
    Vector const wave_delta_right(wave_position_right_ - wave_eepos_task_->getActual());
    wave_distance_right_ = wave_delta_right.norm();
    
    switch (state_) {
      
    case STATE_START:
      if (shake_distance_ > shake_distance_threshold_) {
	shake_count_ = 0;
	state_ = STATE_SHAKE;
      }
      break;
      
    case STATE_SHAKE:
      if (shake_distance_ > shake_distance_threshold_) {
	shake_count_ = 0;
      }
      else {
	++shake_count_;
	if (shake_count_ > shake_count_threshold_) {
	  st = wave_eepos_goal_->set(wave_position_left_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for wave (left): " + st.errstr;
	    return st;
	  }
	  st = wave_posture_goal_->set(wave_posture_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set posture goal for wave: " + st.errstr;
	    return st;
	  }
	  wave_count_ = 0;
	  state_ = STATE_WAVE_LEFT;
	}
      }
      break;

    case STATE_WAVE_LEFT:
      if (wave_distance_left_ < wave_distance_threshold_) {
	++wave_count_;
	if (wave_count_ > wave_count_threshold_) {
	  st = shake_eepos_goal_->set(shake_position_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for shake (return from left): " + st.errstr;
	    return st;
	  }
	  st = shake_posture_goal_->set(shake_posture_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set posture goal for shake (return from left): " + st.errstr;
	    return st;
	  }
	  state_ = STATE_RETURN;
	}
	else {
	  st = wave_eepos_goal_->set(wave_position_right_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for wave (right): " + st.errstr;
	    return st;
	  }
	  state_ = STATE_WAVE_RIGHT;
	}
      }
      break;
      
    case STATE_WAVE_RIGHT:
      if (wave_distance_right_ < wave_distance_threshold_) {
	++wave_count_;
	if (wave_count_ > wave_count_threshold_) {
	  st = shake_eepos_goal_->set(shake_position_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for shake (return from right): " + st.errstr;
	    return st;
	  }
	  st = shake_posture_goal_->set(shake_posture_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set posture goal for shake (return from right): " + st.errstr;
	    return st;
	  }
	  state_ = STATE_RETURN;
	}
	else {
	  st = wave_eepos_goal_->set(wave_position_left_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for wave (left): " + st.errstr;
	    return st;
	  }
	  state_ = STATE_WAVE_LEFT;
	}
      }
      break;

    case STATE_RETURN:
      if (shake_distance_ < 0.75 * shake_distance_threshold_) {
	state_ = STATE_START;
      }
      break;
      
    default:
      return Status(false, "invalid state");
    }
    
    return st;
  }
  
  
  Skill::task_table_t const * HelloGoodbyeSkill::
  getTaskTable()
  {
    switch (state_) {
    case STATE_START:
    case STATE_SHAKE:
    case STATE_RETURN:
      return &shake_task_table_;
    case STATE_WAVE_LEFT:
    case STATE_WAVE_RIGHT:
      return &wave_task_table_;
    }
    return 0;
  }
  
  
  Status HelloGoodbyeSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (//(task == shake_eeori_)
	//||
	(task == shake_eepos_task_)
	|| (task == wave_eepos_task_))
      {
	if (sv.rows() != 3) {
	  return Status(false, "dimension mismatch");
	}
	if (sv[2] < task->getSigmaThreshold()) {
	  return Status(false, "singular");
	}
      }
    Status ok;
    return ok;
  }
  
  
  void HelloGoodbyeSkill::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os, title, prefix);
    os << prefix << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    switch (state_) {
    case STATE_START:
      os << prefix << "  START\n"
	 << prefix << "    shake_distance " << shake_distance_ << "\n";
      break;
    case STATE_SHAKE:
      os << prefix << "  SHAKE\n"
	 << prefix << "    shake_distance " << shake_distance_ << "\n"
	 << prefix << "    shake_count    " << shake_count_ << "\n";
      break;
    case STATE_WAVE_LEFT:
      os << prefix << "  WAVE_LEFT\n"
	 << prefix << "    wave_distance_left " << wave_distance_left_ << "\n"
	 << prefix << "    wave_count         " << wave_count_ << "\n";
      break;
    case STATE_WAVE_RIGHT:
      os << prefix << "  WAVE_RIGHT\n"
	 << prefix << "    wave_distance_right " << wave_distance_right_ << "\n"
	 << prefix << "    wave_count          " << wave_count_ << "\n";
      break;
    case STATE_RETURN:
      os << prefix << "  RETURN\n"
	 << prefix << "    shake_distance " << shake_distance_ << "\n";
      break;
    default:
      os << prefix << "  invalid state " << state_ << "\n";
    }
  }
  
}
