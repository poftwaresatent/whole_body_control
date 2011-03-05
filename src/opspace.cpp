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
		     Matrix & invMatrix,
		     Vector * opt_sigmaOut)
  {
    if ((1 == matrix.rows()) && (1 == matrix.cols())) {
      // workaround for Eigen2
      invMatrix.resize(1, 1);
      if (matrix.coeff(0, 0) > sigmaThreshold) {
	invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
      }
      else {
	invMatrix.coeffRef(0, 0) = 0.0;
      }
      if (opt_sigmaOut) {
	opt_sigmaOut->resize(1);
	opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
      }
      return;
    }
    
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
    if (opt_sigmaOut) {
      *opt_sigmaOut = svd.singularValues();
    }
  }
  
}
