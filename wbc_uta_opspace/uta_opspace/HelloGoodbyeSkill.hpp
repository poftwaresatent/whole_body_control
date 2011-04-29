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

#ifndef UTA_OPSPACE_HELLO_GOODBYE_SKILL_HPP
#define UTA_OPSPACE_HELLO_GOODBYE_SKILL_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class HelloGoodbyeSkill
    : public Skill
  {
  public:
    HelloGoodbyeSkill(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:
    enum {
      STATE_INIT,
      STATE_START,
      STATE_SHAKE,
      STATE_WAVE_LEFT,
      STATE_WAVE_RIGHT,
      STATE_RETURN
    } state_;
    
    JPosTrjTask * init_jpos_task_;
    task_table_t init_task_table_;
    
    //    OrientationTask * shake_eeori_;
    CartPosTrjTask * shake_eepos_task_;
    JPosTrjTask * shake_posture_task_;
    task_table_t shake_task_table_;
    bool shake_needs_init_;
    
    CartPosTrjTask * wave_eepos_task_;
    JPosTrjTask * wave_posture_task_;
    task_table_t wave_task_table_;
    
    Parameter * init_jpos_goal_;
    Parameter * shake_eepos_goal_;
    Parameter * wave_eepos_goal_;
    
    Vector init_jpos_;
    double init_jdist_;
    double init_jdist_threshold_;
    
    Vector shake_position_;
    double shake_distance_;
    double shake_distance_threshold_;
    int shake_count_;
    int shake_count_threshold_;
    
    Vector wave_position_left_;
    Vector wave_position_right_;
    double wave_distance_left_;
    double wave_distance_right_;
    double wave_distance_threshold_;
    int wave_count_;
    int wave_count_threshold_;
  };
  
}

#endif // UTA_OPSPACE_HELLO_GOODBYE_SKILL_HPP
