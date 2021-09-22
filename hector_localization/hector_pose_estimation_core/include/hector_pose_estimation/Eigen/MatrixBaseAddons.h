//=================================================================================================
// Copyright (c) 2014, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_POSE_ESTIMATION_EIGEN_MATRIXBASE_PLUGIN
#define HECTOR_POSE_ESTIMATION_EIGEN_MATRIXBASE_PLUGIN

#ifndef EIGEN_MATRIXBASE_PLUGIN
  #define EIGEN_MATRIXBASE_PLUGIN <hector_pose_estimation/Eigen/MatrixBaseAddons.h>
#endif

#else  // HECTOR_POSE_ESTIMATION_EIGEN_MATRIXBASE_PLUGIN

Derived& setSymmetric() {
  EIGEN_STATIC_ASSERT((RowsAtCompileTime == ColsAtCompileTime) ||
                      (RowsAtCompileTime == Dynamic) ||
                      (ColsAtCompileTime == Dynamic),
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  *this = (*this + this->transpose()) / Scalar(2);
  return derived();
}

Derived& assertSymmetric() {
  EIGEN_STATIC_ASSERT((RowsAtCompileTime == ColsAtCompileTime) ||
                      (RowsAtCompileTime == Dynamic) ||
                      (ColsAtCompileTime == Dynamic),
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
#if defined(ASSERT_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC)
  eigen_assert(this->isApprox(this->transpose(), ASSERT_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC_PRECISION));
#endif
#if defined(FORCE_SYMMETRIC_MATRIX_TO_BE_SYMMETRIC)
  return setSymmetric();
#else
  return derived();
#endif
}

Derived symmetric() const {
  EIGEN_STATIC_ASSERT((RowsAtCompileTime == ColsAtCompileTime) ||
                      (RowsAtCompileTime == Dynamic) ||
                      (ColsAtCompileTime == Dynamic),
                      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
  return (*this + this->transpose()) / Scalar(2);
}

#endif // HECTOR_POSE_ESTIMATION_EIGEN_MATRIXBASE_PLUGIN
