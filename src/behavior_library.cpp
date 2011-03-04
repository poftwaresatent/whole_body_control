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

#include <opspace/behavior_library.hpp>
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace opspace {
  
  
  TPBehavior::
  TPBehavior(std::string const & name)
    : Behavior(name)
  {
    declareSlot("default", "eepos", &eepos_);
    declareSlot("default", "posture", &posture_);
  }
  
  
  Status TPBehavior::
  init(Model const & model)
  {
    Status st(Behavior::init(model));
    if ( ! st) {
      return st;
    }
    task_table_.push_back(eepos_);
    task_table_.push_back(posture_);
    return st;
  }
  
  
  Status TPBehavior::
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
  
  
  Behavior::task_table_t const * TPBehavior::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status TPBehavior::
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
  
  
  HelloGoodbyeBehavior::
  HelloGoodbyeBehavior(std::string const & name)
    : Behavior(name),
      state_(STATE_START),
      //      shake_eeori_(0),
      shake_eepos_(0),
      shake_posture_(0),
      wave_eepos_(0),
      wave_posture_(0),
      shake_eepos_goal_(0),
      wave_eepos_goal_(0),
      shake_position_(Vector::Zero(3)),
      shake_distance_(0.0),
      shake_distance_threshold_(0.1),
      shake_count_(0),
      shake_count_threshold_(200),
      wave_position_left_(Vector::Zero(3)),
      wave_position_right_(Vector::Zero(3)),
      wave_distance_left_(0.0),
      wave_distance_right_(0.0),
      wave_distance_threshold_(0.1),
      wave_count_(0),
      wave_count_threshold_(5)
  {
    //    declareSlot("shake", "orientation", &shake_eeori_);
    declareSlot("shake", "position", &shake_eepos_);
    declareSlot("shake", "posture", &shake_posture_);
    declareSlot("wave", "position", &wave_eepos_);
    declareSlot("wave", "posture", &wave_posture_);
  }
  
  
  Status HelloGoodbyeBehavior::
  init(Model const & model)
  {
    Status st(Behavior::init(model));
    if ( ! st) {
      return st;
    }
    
    shake_eepos_goal_ = shake_eepos_->lookupParameter("goal", PARAMETER_TYPE_VECTOR);
    if ( ! shake_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in shake position task";
      return st;
    }

    wave_eepos_goal_ = wave_eepos_->lookupParameter("goal", PARAMETER_TYPE_VECTOR);
    if ( ! wave_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in wave position task";
      return st;
    }
    
    //    shake_.push_back(shake_eeori_);
    shake_.push_back(shake_eepos_);
    shake_.push_back(shake_posture_);
    wave_.push_back(wave_eepos_);
    wave_.push_back(wave_posture_);
    
    state_ = STATE_START;
    shake_position_ = shake_eepos_->getActual();
    wave_position_left_ = shake_position_;
    wave_position_left_[1] -= 0.1;
    //    wave_position_left_[2] += 0.3;
    wave_position_right_ = shake_position_;
    wave_position_right_[1] += 0.1;
    //    wave_position_right_[2] += 0.3;
    
    return st;
  }
  
  
  Status HelloGoodbyeBehavior::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < shake_.size(); ++ii) {
      st = shake_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    for (size_t ii(0); ii < wave_.size(); ++ii) {
      st = wave_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    
    Vector const shake_delta(shake_position_ - shake_eepos_->getActual());
    shake_distance_ = shake_delta.norm();
    Vector const wave_delta_left(wave_position_left_ - wave_eepos_->getActual());
    wave_distance_left_ = wave_delta_left.norm();
    Vector const wave_delta_right(wave_position_right_ - wave_eepos_->getActual());
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
	    st.errstr = "failed to set position goal for wave (left)";
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
	  state_ = STATE_RETURN;
	}
	else {
	  st = wave_eepos_goal_->set(wave_position_right_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for wave (right)";
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
	  state_ = STATE_RETURN;
	}
	else {
	  st = wave_eepos_goal_->set(wave_position_left_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for wave (left)";
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
  
  
  Behavior::task_table_t const * HelloGoodbyeBehavior::
  getTaskTable()
  {
    switch (state_) {
    case STATE_START:
    case STATE_SHAKE:
    case STATE_RETURN:
      return &shake_;
    case STATE_WAVE_LEFT:
    case STATE_WAVE_RIGHT:
      return &wave_;
    }
    return 0;
  }
  
  
  Status HelloGoodbyeBehavior::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    if (//(task == shake_eeori_)
	//||
	(task == shake_eepos_)
	|| (task == wave_eepos_))
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
  
  
  void HelloGoodbyeBehavior::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Behavior::dbg(os, title, prefix);
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
