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

#include <opspace/TypeIOTGCursor.hpp>

namespace opspace {
  
  
  TypeIOTGCursor::
  TypeIOTGCursor(size_t ndof, double dt_seconds)
    : ndof_(ndof),
      dt_seconds_(dt_seconds),
      otg_(ndof, dt_seconds)
  {
    pos_clean_ = Vector::Zero(ndof);
    vel_clean_ = Vector::Zero(ndof);
    pos_dirty_ = Vector::Zero(ndof);
    vel_dirty_ = Vector::Zero(ndof);
    selection_.resize(ndof);
    for (size_t ii(0); ii < ndof; ++ii) {
      selection_[ii] = true;
    }
  }
  
  
  int TypeIOTGCursor::
  next(Vector const & maxvel,
       Vector const & maxacc,
       Vector const & goal)
  {
    int const result(otg_.GetNextMotionState_Position(pos_clean_.data(),
						      vel_clean_.data(),
						      maxvel.data(),
						      maxacc.data(),
						      goal.data(),
						      selection_.data(),
						      pos_dirty_.data(),
						      vel_dirty_.data()));
    if (0 <= result) {
      pos_clean_ = pos_dirty_;
      vel_clean_ = vel_dirty_;
    }
    return result;
  }
  
}
