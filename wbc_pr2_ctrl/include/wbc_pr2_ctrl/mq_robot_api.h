/*
 * PR2 controller plugin for Stanford-WBC http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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

/**
   \file mq_robot_api.h A wbc::RobotAPI using mqueues for the PR2 controller plugin.
   \author Roland Philippsen
*/

#ifndef WBC_PR2_CTRL_MQ_ROBOT_API_H
#define WBC_PR2_CTRL_MQ_ROBOT_API_H

#include <jspace/RobotAPI.hpp>
#include <stdexcept>
#include <sstream>
#include <mqueue.h>
#include <stdint.h>

namespace wbc_pr2_ctrl {
  
  class MQRobotAPI
    : public jspace::RobotAPI
  {
  public:
    explicit MQRobotAPI(/** Whether to unlink (remove) the mqueue file
			    system entry in the destructor. */
			bool unlink_mqueue);
    
    virtual ~MQRobotAPI();
    
    virtual jspace::Status readState(jspace::State & state);
    
    virtual jspace::Status writeCommand(jspace::Vector const & command);
    
    virtual void shutdown();
    
    void init(/** Say "yes" in the servo and "no" in the pump. */
	      bool blocking,
	      /** Say "blahblah_s2r" in the servo and "blahblah_r2s" in the pump. */
	      std::string const & wr_name,
	      /** Same as \c wr_name but the other way around. */
	      std::string const & rd_name,
	      uint64_t npos, uint64_t nvel, uint64_t nforce, uint64_t ncom) throw(std::runtime_error);
    
    /** \return 0 on SUCESS, 1 on TRY_AGAIN, -1 on ERROR */
    int sendState();
    
    /** \return 0 on SUCESS, 1 on TRY_AGAIN, -1 on ERROR */
    int sendCom();
    
    /** \return 0 on SUCESS, 1 on TRY_AGAIN, -1 on ERROR */
    int receiveState();
    
    /** \return 0 on SUCESS, 1 on TRY_AGAIN, -1 on ERROR */
    int receiveCom();
    
    inline float * pos() { return pos_; }
    inline float * vel() { return vel_; }
    inline float * force() { return force_; }
    inline float * com() { return com_; }
    inline float const * comLatch() const { return com_latch_; }
    inline float * dutyCycle() { return duty_cycle_; }
    inline float dutyCycleLatch() const { return duty_cycle_latch_; }
    
    /** \todo Not quite sure whether this ends up as a pointer to temporary... */
    inline char const * errstr() { return err_os_.str().c_str(); }
    
  protected:
    bool const unlink_mqueue_;
    
    std::string wr_name_, rd_name_;
    uint64_t npos_, nvel_, nforce_, ncom_, state_nbytes_, com_nbytes_;
    
    mqd_t wr_, rd_;
    std::ostringstream err_os_;
    
    uint64_t * check_npos_, * check_nvel_, * check_nforce_, * check_ncom_;
    float * pos_, * vel_, * force_, * duty_cycle_, * com_;
    char * buffer_;
    float * com_latch_;
    float duty_cycle_latch_;
  };
  
}

#endif // WBC_PR2_CTRL_MQ_ROBOT_API_H
