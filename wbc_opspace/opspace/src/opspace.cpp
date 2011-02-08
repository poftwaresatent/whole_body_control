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

#include <opspace/opspace.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

using namespace std;

namespace opspace {

  void pseudoInverse(Matrix const & matrix,
		     double sigmaThreshold,
		     Matrix & invMatrix)
  {
    Eigen::SVD<Matrix> svd(matrix);
    // not sure if we need to svd.sort()... probably not
    int const nrows(svd.singularValues().rows());
    Matrix invS;
    invS = Matrix::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii) {
      if (svd.singularValues().coeff(ii) > sigmaThreshold) {
	invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
      }
    }
    invMatrix = svd.matrixU() * invS * svd.matrixU().transpose();
  }
  
  
  void computeTaskMatrices(Matrix const * nullspace_in,
			   Matrix const * jacobian,
			   Matrix const & invMassInertia,
			   Matrix & lambda,
			   Matrix & jstar,
			   Matrix * nullspace_out)
  {
    if (nullspace_in) {
      if (jacobian) {
	jstar = (*jacobian) * (*nullspace_in);
      }
      else {
	jstar = *nullspace_in;
      }
    }
    else {
      if (jacobian) {
	jstar = *jacobian;
      }
      else {
	// this case is a complete waste, because the caller should
	// just directly get the mass inertia from the model... won't
	// happen too often, hopefully
	jstar = Matrix::Identity(invMassInertia.cols(), invMassInertia.rows());
      }
    }
    
    Matrix const invLambda(jstar * invMassInertia * jstar.transpose());
    pseudoInverse(invLambda, 1e-3, lambda);
    
    if (nullspace_out) {
      if (( ! jacobian) && ( ! nullspace_in)) {
	*nullspace_out = Matrix::Zero(invMassInertia.rows(), invMassInertia.cols());
      }
      else {
	*nullspace_out = Matrix::Identity(invMassInertia.rows(), invMassInertia.cols());
	if (nullspace_in) {
	  *nullspace_out -= (invMassInertia * jstar.transpose() * lambda * jstar) * (*nullspace_in);
	}
	else {
	  *nullspace_out -= invMassInertia * jstar.transpose() * lambda * jstar;
	}
      }
    }
  }
  
  
  TaskAccumulator::
  TaskAccumulator(Matrix const & invMassInertia)
    : invMassInertia_(invMassInertia),
      ndof_(invMassInertia_.rows()),
      command_(ndof_)
  {
  }
  
  
  TaskAccumulator::
  ~TaskAccumulator()
  {
    for (matrix_table_t::iterator ii(lambda_table_.begin());
	 ii != lambda_table_.end(); ++ii) {
      delete *ii;
    }
    for (matrix_table_t::iterator ii(jstar_table_.begin());
	 ii != jstar_table_.end(); ++ii) {
      delete *ii;
    }
    for (matrix_table_t::iterator ii(nullspace_table_.begin());
	 ii != nullspace_table_.end(); ++ii) {
      delete *ii;
    }
  }
  
  
  size_t TaskAccumulator::
  addTask(Vector const & acceleration,
	  Matrix const & jacobian)
  {
    Matrix * lambda(new Matrix());
    Matrix * jstar(new Matrix());
    Matrix * nullspace(new Matrix());
    size_t const level(lambda_table_.size());
    
    if (0 == level) {

      // First iteration, nullspace_in is identity, which we can
      // signal by passing a zero pointer.
      
      computeTaskMatrices(0, &jacobian, invMassInertia_, *lambda, *jstar, nullspace);
      command_ = jacobian.transpose() * (*lambda) * acceleration;
      
    }
    else {
      
#warning CHECK JSTAR STUFF WITH LUIS
#warning CHECK JSTAR STUFF WITH LUIS
#warning CHECK JSTAR STUFF WITH LUIS

      Matrix const * prev_nullspace(nullspace_table_[level - 1]);
      
      computeTaskMatrices(prev_nullspace, &jacobian, invMassInertia_, *lambda, *jstar, nullspace);

      Matrix jstar;
      jstar = jacobian * (*prev_nullspace);
      command_ = jstar.transpose() * (*lambda) * acceleration;
      
    }
    
    lambda_table_.push_back(lambda);
    jstar_table_.push_back(jstar);
    nullspace_table_.push_back(nullspace);
    
    return level;
  }
  
  
  Matrix const * TaskAccumulator::
  getLambda(size_t level) const
  {
    if (level >= lambda_table_.size()) {
      return 0;
    }
    return lambda_table_[level];
  }
  
  
  Matrix const * TaskAccumulator::
  getJstar(size_t level) const
  {
    if (level >= jstar_table_.size()) {
      return 0;
    }
    return jstar_table_[level];
  }
  
  
  Matrix const * TaskAccumulator::
  getNullspace(size_t level) const
  {
    if (level >= nullspace_table_.size()) {
      return 0;
    }
    return nullspace_table_[level];
  }
  
}
