/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2010 University of Texas at Austin. All rights reserved.
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

#include <jspace/wrap_eigen.hpp>

namespace opspace {
  
  using jspace::Matrix;
  using jspace::Vector;
  
  /**
     This pseudo-inverse is based on SVD, followed by threshlding on
     the singular values.
  */
  void pseudoInverse(Matrix const & matrix,
		     double sigmaThreshold,
		     Matrix & invMatrix);
  
  void computeTaskMatrices(Matrix const * nullspace_in,
			   Matrix const * jacobian,
			   Matrix const & invMassInertia,
			   Matrix & lambda,
			   Matrix & jstar,
			   Matrix * nullspace_out);
  
  class TaskAccumulator
  {
  public:
    explicit TaskAccumulator(/** \note This reference needs to stay
				 valid for the lifetime of the
				 TaskAccumulator. */
			     Matrix const & invMassInertia);
    virtual ~TaskAccumulator();
    
    size_t addTask(Vector const & acceleration, Matrix const & jacobian);
    
    Vector const & getFinalCommand() const { return command_; }
    size_t getNLevels() const { return lambda_table_.size(); }
    Matrix const * getLambda(size_t level) const;
    Matrix const * getJstar(size_t level) const;
    Matrix const * getNullspace(size_t level) const;
    
  protected:
    typedef std::vector<Matrix *> matrix_table_t;
    
    Matrix const & invMassInertia_;
    int const ndof_;
    
    Vector command_;
    matrix_table_t lambda_table_;
    matrix_table_t jstar_table_;
    matrix_table_t nullspace_table_;
  };
  
}
