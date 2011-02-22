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

#ifndef OPSPACE_OTHER_CONTROLLERS_HPP
#define OPSPACE_OTHER_CONTROLLERS_HPP

#include <opspace/Controller.hpp>

namespace opspace {


  /**
     A controller implementation based on "Samir's cheatsheet" ... not
     thoroughly tested yet, but if it turns out to work correctly, it
     would probably be faster than the LController.
  */
  class SController
    : public Controller
  {
  public:
    explicit SController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status computeCommand(Model const & model, Vector & gamma);
  };
  
  
  /**
     Just a test controller in case you know you have exactly two
     tasks, with the lower-level one being a full jointspace posture
     task. Will probably be completely removed "real soon now".
   */
  class TPController
    : public Controller
  {
  public:
    explicit TPController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status init(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & gamma);
    
  protected:
    Task * task_;
    Task * posture_;
  };
  
  
  /**
     Quick test for using algebraic regularization for lambda... does
     not work though (yet).
  */
  class LRController
    : public Controller
  {
  public:
    explicit LRController(std::string const & name, std::ostream * dbg = 0);
    
    virtual Status computeCommand(Model const & model, Vector & gamma);
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    std::vector<Vector> sv_lstar_;
    std::vector<Vector> sv_jstar_;
  };

}

#endif // OPSPACE_OTHER_CONTROLLERS_HPP
