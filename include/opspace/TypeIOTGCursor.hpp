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

#ifndef OPSPACE_TYPE_I_OTG_CURSOR_HPP
#define OPSPACE_TYPE_I_OTG_CURSOR_HPP

#include <jspace/wrap_eigen.hpp>
#include <reflexxes_otg/TypeIOTG.h>

namespace opspace {
  
  using jspace::Vector;
  

  /**
     Utility for using reflexxes_otg::TypeIOTG. This class wraps a
     acceleration-bounded trajectory object from the reflexxes_otg
     library and makes it easier to use, e.g. inside
     opspace::TrajectoryTask.
     
     The idea is that you simply initialize it by setting the starting
     position() and velocity(), and then repeatedly call next() to
     advance to the next desired position and velocity.
  */
  class TypeIOTGCursor
  {
  public:
    size_t const ndof_;
    double const dt_seconds_;
    
    TypeIOTGCursor(size_t ndof,
		   double dt_seconds);
    
    /**
       Compute the next desired position() and velocity(). You specify
       the maximum velocity and acceleration right here, as well as
       the goal. This allows you to change all these parameters on the
       fly without needing to mess with the cursor's internal state.
       
       If you want to change the starting state of the trajectory,
       simply assign to position() and velocity(). You typically do
       that only at (re-)initialization time.
       
       \return reflexxes_otg::TypeIOTGResult which is easy to
       interpret: 0 on success, 1 if we were already at the goal
       anyway, and negative values on error.
    */
    int next(Vector const & maxvel,
	     Vector const & maxacc,
	     Vector const & goal);
    
    inline Vector & position()             { return pos_clean_; }
    inline Vector const & position() const { return pos_clean_; }
    inline Vector & velocity()             { return vel_clean_; }
    inline Vector const & velocity() const { return vel_clean_; }
    
  protected:
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> boolvec_t;
    
    TypeIOTG otg_;
    boolvec_t selection_;
    Vector pos_clean_;
    Vector vel_clean_;
    Vector pos_dirty_;
    Vector vel_dirty_;
  };
  
}

#endif // OPSPACE_TYPE_I_OTG_CURSOR_HPP
