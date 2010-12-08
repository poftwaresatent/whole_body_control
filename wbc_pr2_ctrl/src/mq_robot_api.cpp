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
   \file mq_robot_api.cpp A wbc::RobotAPI using mqueues for the PR2 controller plugin.
   \author Roland Philippsen
*/

#include <wbc_pr2_ctrl/mq_robot_api.h>
#include <ros/ros.h>
#include <sys/time.h>
#include <err.h>
#include <errno.h>

using namespace std;


namespace wbc_pr2_ctrl {
  
  
  MQRobotAPI::
  MQRobotAPI(bool unlink_mqueue)
    : unlink_mqueue_(unlink_mqueue),
      wr_(-1),
      rd_(-1),
      check_npos_(0),
      check_nvel_(0),
      check_nforce_(0),
      check_ncom_(0),
      pos_(0),
      vel_(0),
      force_(0),
      duty_cycle_(0),
      com_(0),
      buffer_(0),
      com_latch_(0),
      duty_cycle_latch_(0)
  {
  }
  
  
  MQRobotAPI::
  ~MQRobotAPI()
  {
    if (-1 != wr_) {
      if (0 != mq_close(wr_)) {
	warn("mq_close %s", wr_name_.c_str());
      }
      if (unlink_mqueue_) {
	if (0 != mq_unlink(wr_name_.c_str())) {
	  warn("mq_unlink %s", wr_name_.c_str());
	}
      }
    }
    if ( -1 != rd_) {
      if (0 != mq_close(rd_)) {
	warn("mq_close %s", rd_name_.c_str());
      }
      if (unlink_mqueue_) {
	if (0 != mq_unlink(rd_name_.c_str())) {
	  warn("mq_unlink %s", rd_name_.c_str());
	}
      }
    }
    delete[] buffer_;
    delete[] com_latch_;
  }
  
  
  jspace::Status MQRobotAPI::
  readState(jspace::State & state)
  {
    jspace::Status status;
    
    if (0 > receiveState()) {
      ROS_ERROR ("wbc_pr2_ctrl::MQRobotAPI::readState(): receiveState() failed: %s", errstr());
      status.ok = false;
      status.errstr = "receiveState() failed";
      return status;
    }
    
    state.position_.resize(npos_);
    for (uint64_t ii(0); ii < npos_; ++ii)
      state.position_[ii] = pos_[ii];
    
    state.velocity_.resize(nvel_);
    for (uint64_t ii(0); ii < nvel_; ++ii)
      state.velocity_[ii] = vel_[ii];
    
    state.force_.resize(nforce_);
    for (uint64_t ii(0); ii < nforce_; ++ii)
      state.force_[ii] = force_[ii];
    
    struct timeval now;
    if (0 != gettimeofday(&now, NULL)) {
      state.time_sec_ = 0;
      state.time_usec_ = 0;
    }
    else {
      state.time_sec_ = now.tv_sec;
      state.time_usec_ = now.tv_usec;
    }
    
    return status;
  }
  
  
  jspace::Status MQRobotAPI::
  writeCommand(jspace::Vector const & command)
  {
    jspace::Status status;
    
    if (static_cast<uint64_t>(command.size()) != ncom_) {
      ROS_ERROR ("wbc_pr2_ctrl::MQRobotAPI::writeCommand(): command.size() is %d but should be %llu",
		 command.size(), ncom_);
      status.ok = false;
      status.errstr = "command.size() mismatch";
      return status;
    }
    
    for (uint64_t ii(0); ii < ncom_; ++ii)
      com_[ii] = command[ii];
    
    if (0 > sendCom()) {
      ROS_ERROR ("wbc_pr2_ctrl::MQRobotAPI::writeCommand(): sendCom() failed: %s", errstr());
      status.ok = false;
      status.errstr = "sendCom() failed";
      return status;
    }
    
    return status;
  }
  
  
  void MQRobotAPI::
  shutdown()
  {
  }
  
  
  void MQRobotAPI::
  init(bool blocking,
       std::string const & wr_name, std::string const & rd_name, 
       uint64_t npos, uint64_t nvel, uint64_t nforce, uint64_t ncom)
    throw(std::runtime_error)
  {
    if (buffer_)
      throw runtime_error("MQRobotAPI cannot be initialized twice");
    if (wr_name.empty() || rd_name.empty())
      throw runtime_error("MQRobotAPI cannot handle empty queue names");
    
    if ('/' == wr_name[0])
      wr_name_ = wr_name;
    else
      wr_name_ = "/" + wr_name;
    if ('/' == rd_name[0])
      rd_name_ = rd_name;
    else
      rd_name_ = "/" + rd_name;
    
    npos_ = npos;
    nvel_ = nvel;
    nforce_ = nforce;
    ncom_ = ncom;
    state_nbytes_ = 4 * sizeof(uint64_t) + (npos + nvel + nforce + 1) * sizeof(float);
    com_nbytes_ = sizeof(uint64_t) + ncom * sizeof(float);
    
    mq_attr attr;
    attr.mq_curmsgs = 0;
    if (blocking)
      attr.mq_flags = 0;
    else
      attr.mq_flags = O_NONBLOCK;
    attr.mq_maxmsg = 1;
    
    attr.mq_msgsize = state_nbytes_;
    if (blocking)
      wr_ = mq_open(wr_name_.c_str(), O_CREAT | O_WRONLY, S_IRUSR | S_IWUSR, &attr);
    else
      wr_ = mq_open(wr_name_.c_str(), O_CREAT | O_NONBLOCK | O_WRONLY, S_IRUSR | S_IWUSR, &attr);
    if (-1 == wr_)
      throw runtime_error("MQRobotAPI mq_open " + wr_name_ + ": " + strerror(errno));
    
    attr.mq_msgsize = com_nbytes_;
    if (blocking)
      rd_ = mq_open(rd_name_.c_str(), O_CREAT | O_RDONLY, S_IRUSR | S_IWUSR, &attr);
    else
      rd_ = mq_open(rd_name_.c_str(), O_CREAT | O_NONBLOCK | O_RDONLY, S_IRUSR | S_IWUSR, &attr);
    if (-1 == rd_)
      throw runtime_error("MQRobotAPI mq_open " + rd_name_ + ": " + strerror(errno));
    
    size_t const buf_nbytes(com_nbytes_ > state_nbytes_ ? com_nbytes_ : state_nbytes_);
    buffer_ = new char[buf_nbytes];
    memset(buffer_, 0, buf_nbytes);
    
    com_latch_ = new float[ncom];
    memset(com_latch_, 0, sizeof(float) * ncom);
    
    check_npos_ = (uint64_t *) buffer_;
    check_nvel_ = check_npos_ + 1;
    check_nforce_ = check_nvel_ + 1;
    pos_ = (float*) (check_nforce_ + 1);
    vel_ = pos_ + npos_;
    force_ = vel_ + nvel_;
    duty_cycle_ = force_ + nforce_;
    
    check_ncom_ = (uint64_t *) buffer_;
    com_ = (float*) (check_ncom_ + 1);
  }
  
  
  int MQRobotAPI::
  sendState()
  {
    if ( ! buffer_) {
      err_os_ << "MQRobotAPI::sendState(): not initialized\n";
      return -1;
    }
    
    *check_npos_ = npos_;
    *check_nvel_ = nvel_;
    *check_nforce_ = nforce_;
    
    if (0 != mq_send(wr_, buffer_, state_nbytes_, 0)) {
      if (EAGAIN == errno)
	return 1;
      err_os_ << "MQRobotAPI::sendState(): mq_send " << wr_name_ << ": " << strerror(errno) << "\n";
      return -1;
    }
    
    return 0;
  }
  
  
  int MQRobotAPI::
  sendCom()
  {
    if ( ! buffer_) {
      err_os_ << "MQRobotAPI::sendCom(): not initialized\n";
      return -1;
    }
    
    *check_ncom_ = ncom_;
    
    if (0 != mq_send(wr_, buffer_, com_nbytes_, 0)) {
      if (EAGAIN == errno)
	return 1;
      err_os_ << "MQRobotAPI::sendCom(): mq_send " << wr_name_ << ": " << strerror(errno) << "\n";
      return -1;
    }
    
    return 0;
  }
  
  
  int MQRobotAPI::
  receiveState()
  {
    if ( ! buffer_) {
      err_os_ << "MQRobotAPI::receiveState(): not initialized\n";
      return -1;
    }
    
    if (-1 == mq_receive(rd_, buffer_, state_nbytes_, 0)) {
      if (EAGAIN == errno)
	return 1;
      err_os_ << "MQRobotAPI::receiveState(): mq_receive " << rd_name_ << ": " << strerror(errno) << "\n";
      return -1;
    }
    
    if (*check_npos_ != npos_) {
      err_os_ << "MQRobotAPI::receiveState(): npos is " << npos_ << " but check_npos is " << *check_npos_ << "\n";
      return -1;
    }
    
    if (*check_nvel_ != nvel_) {
      err_os_ << "MQRobotAPI::receiveState(): nvel is " << nvel_ << " but check_nvel is " << *check_nvel_ << "\n";
      return -1;
    }
    
    if (*check_nforce_ != nforce_) {
      err_os_ << "MQRobotAPI::receiveState(): nforce is " << nforce_ << " but check_nforce is " << *check_nforce_
	      << "\n";
      return -1;
    }
    
    duty_cycle_latch_ = *duty_cycle_;
    
    return 0;
  }
  
  
  int MQRobotAPI::
  receiveCom()
  {
    if ( ! buffer_) {
      err_os_ << "MQRobotAPI::receiveCom(): not initialized\n";
      return -1;
    }
    
    if (-1 == mq_receive(rd_, buffer_, com_nbytes_, 0)) {
      if (EAGAIN == errno)
	return 1;
      err_os_ << "MQRobotAPI::receiveCom(): mq_receive " << rd_name_ << ": " << strerror(errno) << "\n";
      return -1;
    }
    
    if (*check_ncom_ != ncom_) {
      err_os_ << "MQRobotAPI::receiveCom(): ncom is " << ncom_ << " but check_ncom is " << *check_ncom_ << "\n";
      return -1;
    }
    
    memcpy(com_latch_, com_, sizeof(float) * ncom_);
    
    return 0;
  }
  
}
