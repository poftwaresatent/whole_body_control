/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include "HelloGoodbyeSkill.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  HelloGoodbyeSkill::
  HelloGoodbyeSkill(std::string const & name)
    : Skill(name),
      state_(STATE_INIT),
      init_jpos_task_(0),
      shake_eepos_task_(0),
      shake_posture_task_(0),
      wave_eepos_task_(0),
      wave_posture_task_(0),
      init_jpos_goal_(0),
      shake_eepos_goal_(0),
      wave_eepos_goal_(0),
      init_jdist_threshold_(-1),
      shake_distance_threshold_(-1),
      shake_count_threshold_(-1),
      wave_distance_threshold_(-1),
      wave_count_threshold_(-1)
  {
    declareSlot("init_jpos", &init_jpos_task_);
    declareParameter("init_jpos", &init_jpos_);
    declareParameter("init_jdist", &init_jdist_, opspace::PARAMETER_FLAG_READONLY);
    declareParameter("init_jdist_threshold", &init_jdist_threshold_);
    
    declareSlot("shake_position", &shake_eepos_task_);
    declareSlot("shake_posture", &shake_posture_task_);
    declareParameter("shake_position", &shake_position_);
    declareParameter("shake_distance", &shake_distance_, opspace::PARAMETER_FLAG_READONLY);
    declareParameter("shake_distance_threshold", &shake_distance_threshold_);
    declareParameter("shake_count", &shake_count_, opspace::PARAMETER_FLAG_READONLY);
    declareParameter("shake_count_threshold", &shake_count_threshold_);

    declareSlot("wave_position", &wave_eepos_task_);
    declareSlot("wave_posture", &wave_posture_task_);
    declareParameter("wave_position_left", &wave_position_left_);
    declareParameter("wave_position_right", &wave_position_right_);
    declareParameter("wave_distance_left", &wave_distance_left_, opspace::PARAMETER_FLAG_READONLY);
    declareParameter("wave_distance_right", &wave_distance_right_, opspace::PARAMETER_FLAG_READONLY);
    declareParameter("wave_distance_threshold", &wave_distance_threshold_);
    declareParameter("wave_count", &wave_count_, opspace::PARAMETER_FLAG_READONLY);
    declareParameter("wave_count_threshold", &wave_count_threshold_);
  }
  
  
  Status HelloGoodbyeSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    
    //////////////////////////////////////////////////
    // init parameter channels
    
    // XXXX to do: could read the name of the parameter from a
    // parameter (also for other tasks)
    init_jpos_goal_ = init_jpos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! init_jpos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in shake position task";
      return st;
    }
    
    shake_eepos_goal_ = shake_eepos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! shake_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in shake position task";
      return st;
    }
    
    wave_eepos_goal_ = wave_eepos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! wave_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in wave position task";
      return st;
    }
    
    //////////////////////////////////////////////////
    // init task tables
    
    init_task_table_.push_back(init_jpos_task_);
    
    shake_task_table_.push_back(shake_eepos_task_);
    shake_task_table_.push_back(shake_posture_task_);
    
    wave_task_table_.push_back(wave_eepos_task_);
    wave_task_table_.push_back(wave_posture_task_);
    
    //////////////////////////////////////////////////
    // check / initialize task parameters
    
    if (7 != init_jpos_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing init_jpos parameter";
      return st;
    }
    
    if (0 > init_jdist_threshold_) {
      init_jdist_threshold_ = 10.0 * M_PI / 180.0;
    }
    
    if (3 != shake_position_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing shake_position parameter";
      return st;
    }
    
    if (0 > shake_distance_threshold_) {
      shake_distance_threshold_ = 0.08;
    }
    
    if (0 > shake_count_threshold_) {
      shake_count_threshold_ = 700; // number of ticks in timeout
    }
    
    if (3 != wave_position_left_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing wave_position_left parameter";
      return st;
    }
    
    if (3 != wave_position_right_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing wave_position_right parameter";
      return st;
    }
    
    if (0 > wave_distance_threshold_) {
      wave_distance_threshold_ = 0.12;
    }
    
    if (0 > wave_count_threshold_) {
      wave_count_threshold_ = 6; // number of times we switch waving direction
    }

    //////////////////////////////////////////////////
    // set those goals which we need at the very beginning, others get
    // filled in as we transition through the state machine... for
    // which we need lazy init flags.
    
    st = init_jpos_goal_->set(init_jpos_);
    if ( ! st) {
      return st;
    }
    
    shake_needs_init_ = true;
    
    //////////////////////////////////////////////////
    // ready to rock
    
    state_ = STATE_INIT;
    return st;
  }
  
  
  Status HelloGoodbyeSkill::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < init_task_table_.size(); ++ii) {
      st = init_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
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
    
    Vector const init_delta(init_jpos_ - init_jpos_task_->getActual());
    init_jdist_ = 0;
    for (int ii(0); ii < init_delta.rows(); ++ii) {
      double const dd(fabs(init_delta[ii]));
      if (dd > init_jdist_) {
	init_jdist_ = dd;
      }
    }
    Vector const shake_delta(shake_position_ - shake_eepos_task_->getActual());
    shake_distance_ = shake_delta.norm();
    Vector const wave_delta_left(wave_position_left_ - wave_eepos_task_->getActual());
    wave_distance_left_ = wave_delta_left.norm();
    Vector const wave_delta_right(wave_position_right_ - wave_eepos_task_->getActual());
    wave_distance_right_ = wave_delta_right.norm();
    
    switch (state_) {
      
    case STATE_INIT:
      if (init_jdist_ <= init_jdist_threshold_) {
	if (shake_needs_init_) {
	  for (size_t ii(0); ii < shake_task_table_.size(); ++ii) {
	    st = shake_task_table_[ii]->init(model);
	    if ( ! st) {
	      return st;
	    }
	  }
	  st = shake_eepos_goal_->set(shake_position_);
	  if ( ! st) {
	    return st;
	  }
	  shake_needs_init_ = false;
	}
	state_ = STATE_START;
      }
      break;
      
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
	  // always re-init the wave trajectory start point
	  for (size_t ii(0); ii < wave_task_table_.size(); ++ii) {
	    st = wave_task_table_[ii]->init(model);
	    if ( ! st) {
	      return st;
	    }
	  }
	  st = wave_eepos_goal_->set(wave_position_right_);
	  if ( ! st) {
	    st.ok = false;
	    st.errstr = "failed to set position goal for wave (right): " + st.errstr;
	    return st;
	  }
	  wave_count_ = 0;
	  state_ = STATE_WAVE_RIGHT;
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
    case STATE_INIT:
      return &init_task_table_;
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
    case STATE_INIT:
      os << prefix << "  INIT\n";
      for (size_t ii(0); ii < init_task_table_.size(); ++ii) {
	init_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      os << prefix << "    init_jdist " << init_jdist_ << "\n";
      break;
    case STATE_START:
      os << prefix << "  START\n";
      for (size_t ii(0); ii < shake_task_table_.size(); ++ii) {
	shake_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      os << prefix << "    shake_distance " << shake_distance_ << "\n";
      break;
    case STATE_SHAKE:
      os << prefix << "  SHAKE\n";
      for (size_t ii(0); ii < shake_task_table_.size(); ++ii) {
	shake_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      os << prefix << "    shake_distance " << shake_distance_ << "\n"
	 << prefix << "    shake_count    " << shake_count_ << "\n";
      break;
    case STATE_WAVE_LEFT:
      os << prefix << "  WAVE_LEFT\n";
      for (size_t ii(0); ii < wave_task_table_.size(); ++ii) {
	wave_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      os << prefix << "    wave_distance_left " << wave_distance_left_ << "\n"
	 << prefix << "    wave_count         " << wave_count_ << "\n";
      break;
    case STATE_WAVE_RIGHT:
      os << prefix << "  WAVE_RIGHT\n";
      for (size_t ii(0); ii < wave_task_table_.size(); ++ii) {
	wave_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      os << prefix << "    wave_distance_right " << wave_distance_right_ << "\n"
	 << prefix << "    wave_count          " << wave_count_ << "\n";
      break;
    case STATE_RETURN:
      os << prefix << "  RETURN\n";
      for (size_t ii(0); ii < shake_task_table_.size(); ++ii) {
	shake_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      os << prefix << "    shake_distance " << shake_distance_ << "\n";
      break;
    default:
      os << prefix << "  invalid state " << state_ << "\n";
    }
  }
  
}
