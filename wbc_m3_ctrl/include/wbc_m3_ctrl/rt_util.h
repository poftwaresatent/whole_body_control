/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Author: Roland Philippsen
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

#ifndef WBC_M3_CTRL_RT_UTIL_H
#define WBC_M3_CTRL_RT_UTIL_H

#include <jspace/State.hpp>
#include <stdexcept>


namespace wbc_m3_ctrl {
  
  
  typedef enum {
    RT_THREAD_UNDEF,
    RT_THREAD_INIT,
    RT_THREAD_RUNNING,
    RT_THREAD_CLEANUP,
    RT_THREAD_ERROR,
    RT_THREAD_DONE
  } rt_thread_state_t;
  
  
  class RTUtil
  {
  public:
    virtual ~RTUtil();
    
    virtual int init(jspace::State const & state) = 0;
    
    virtual int update(jspace::State const & state,
		       jspace::Vector & command) = 0;
    
    virtual int cleanup(void) = 0;
    
    virtual int slowdown(long long iteration,
			 long long desired_ns,
			 long long actual_ns) = 0;
    
    void start(long long tick_frequency_hz) throw(std::runtime_error);
    
    static rt_thread_state_t getState();
    static rt_thread_state_t shutdown();
  };
  
}

#endif // WBC_M3_CTRL_RT_UTIL_H
