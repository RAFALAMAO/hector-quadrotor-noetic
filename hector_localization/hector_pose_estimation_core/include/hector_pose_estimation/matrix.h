//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_MATRIX_H
#define HECTOR_POSE_ESTIMATION_MATRIX_H

#include <hector_pose_estimation/matrix_config.h>
#include <hector_pose_estimation/Eigen/MatrixBaseAddons.h>
#include <hector_pose_estimation/Eigen/QuaternionBaseAddons.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>

namespace hector_pose_estimation {
  using Eigen::Dynamic;
  using Eigen::Lower;
  using Eigen::Upper;
  using Eigen::DenseBase;

  typedef double ScalarType;
  typedef Eigen::DenseIndex IndexType;

  typedef Eigen::Quaternion<ScalarType> Quaternion;

  template <int Rows>
  struct ColumnVector_ {
    typedef Eigen::Matrix<ScalarType, Rows, 1,
                          Eigen::ColMajor,
                          (Rows != Dynamic ? Rows : MaxVectorSize), 1
                         > type;
  };
  typedef typename ColumnVector_<Dynamic>::type ColumnVector;
  typedef typename ColumnVector_<3>::type ColumnVector3;

  template <int Cols>
  struct RowVector_ {
      typedef Eigen::Matrix<ScalarType, 1, Cols,
                            Eigen::RowMajor,
                            1, (Cols != Dynamic ? Cols : MaxVectorSize)
                           > type;
  };
  typedef typename RowVector_<Dynamic>::type RowVector;
  typedef typename RowVector_<3>::type RowVector3;

  template <int Rows, int Cols>
  struct Matrix_ {
      typedef Eigen::Matrix<ScalarType, Rows, Cols,
                            (Rows == 1 && Cols != 1 ? Eigen::RowMajor : Eigen::ColMajor),
                            (Rows != Dynamic ? Rows : MaxMatrixRowsCols),
                            (Cols != Dynamic ? Cols : MaxMatrixRowsCols)
                           > type;
  };
  typedef typename Matrix_<Dynamic,Dynamic>::type Matrix;
  typedef typename Matrix_<3,3>::type Matrix3;

  template <int RowsCols>
  struct SymmetricMatrix_ {
      typedef typename Matrix_<RowsCols,RowsCols>::type type;
  };
  typedef typename SymmetricMatrix_<3>::type SymmetricMatrix3;
  typedef typename SymmetricMatrix_<6>::type SymmetricMatrix6;
  typedef typename SymmetricMatrix_<Dynamic>::type SymmetricMatrix;

  template <typename OtherDerived>
  static inline Matrix3 SkewSymmetricMatrix(const Eigen::MatrixBase<OtherDerived>& other) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OtherDerived>, 3);
    Matrix3 result;
    result <<  0.0,       -other.z(),  other.y(),
               other.z(),  0.0,       -other.x(),
              -other.y(),  other.x(),  0.0;
    return result;
  }

} // namespace hector_pose_estimation

namespace Eigen {

// Add template specializations of DenseStorage for null matrices for older Eigen versions.
// Those have been added to Eigen 3.1.0.
// See https://bitbucket.org/eigen/eigen/commits/e03061319d0297c34e3128dcc5a780824b4fdb78.
// Copied from https://bitbucket.org/eigen/eigen/src/167ce78594dc4e7a4b9ca27fc745e674300e85ff/Eigen/src/Core/DenseStorage.h.
#if !EIGEN_VERSION_AT_LEAST(3,0,91)
  // more specializations for null matrices; these are necessary to resolve ambiguities
  template<typename T, int _Options> class DenseStorage<T, 0, Dynamic, Dynamic, _Options>
  : public DenseStorage<T, 0, 0, 0, _Options> { };

  template<typename T, int _Rows, int _Options> class DenseStorage<T, 0, _Rows, Dynamic, _Options>
  : public DenseStorage<T, 0, 0, 0, _Options> { };

  template<typename T, int _Cols, int _Options> class DenseStorage<T, 0, Dynamic, _Cols, _Options>
  : public DenseStorage<T, 0, 0, 0, _Options> { };
#endif

} // namespace Eigen

namespace hector_pose_estimation {

  using Eigen::VectorBlock;
  typedef VectorBlock<ColumnVector,3>       VectorBlock3;
  typedef VectorBlock<ColumnVector,4>       VectorBlock4;
  typedef VectorBlock<const ColumnVector,3> ConstVectorBlock3;
  typedef VectorBlock<const ColumnVector,4> ConstVectorBlock4;

  using Eigen::Block;
  typedef Eigen::Block<Matrix,Dynamic,Dynamic> MatrixBlock;

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MATRIX_H
