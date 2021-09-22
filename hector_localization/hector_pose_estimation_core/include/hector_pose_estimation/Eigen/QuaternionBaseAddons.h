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

#ifndef HECTOR_POSE_ESTIMATION_EIGEN_QUATERNIONBASE_PLUGIN
#define HECTOR_POSE_ESTIMATION_EIGEN_QUATERNIONBASE_PLUGIN

#ifndef EIGEN_QUATERNIONBASE_PLUGIN
  #define EIGEN_QUATERNIONBASE_PLUGIN <hector_pose_estimation/Eigen/QuaternionBaseAddons.h>
#endif

#else  // HECTOR_POSE_ESTIMATION_EIGEN_QUATERNIONBASE_PLUGIN

template <typename OtherDerived>
Derived &fromRotationVector(const MatrixBase<OtherDerived> &rotation_vector)
{
  eigen_assert(rotation_vector.size() == 3);

  const double angle = rotation_vector.norm();
  double sin_angle_2, cos_angle_2;
  ::sincos(angle / 2., &sin_angle_2, &cos_angle_2);
  double sin_angle_norm_2 = 0.5;
  if (angle > NumTraits<double>::dummy_precision()) sin_angle_norm_2 = sin_angle_2 / angle;

  w() = cos_angle_2;
  vec() = rotation_vector * sin_angle_norm_2;

  return derived();
}

Vector3 toRotationVector() const
{
  eigen_assert(squaredNorm() == 1.0);
  Scalar vector_norm = vec().norm();
  if (vector_norm <= NumTraits<Scalar>::dummy_precision())
    return vec() * Scalar(2.);
  else
    return vec() / vector_norm * ::acos(w()) * Scalar(2.);
}

#endif // HECTOR_POSE_ESTIMATION_EIGEN_QUATERNIONBASE_PLUGIN
