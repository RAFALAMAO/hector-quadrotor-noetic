//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_BARO_H
#define HECTOR_POSE_ESTIMATION_BARO_H

#include <hector_pose_estimation/measurement.h>
#include <hector_pose_estimation/measurements/height.h>

#ifdef USE_HECTOR_UAV_MSGS
  #include <hector_uav_msgs/Altimeter.h>
#endif

namespace hector_pose_estimation {

class BaroUpdate;

class BaroModel : public HeightModel
{
public:
  BaroModel();
  virtual ~BaroModel();

  virtual void getExpectedValue(MeasurementVector& y_pred, const State& state);
  virtual void getStateJacobian(MeasurementMatrix& C, const State& state, bool init);

  void setQnh(double qnh) { qnh_ = qnh; }
  double getQnh() const { return qnh_; }

  double getAltitude(const BaroUpdate& update);

protected:
  double qnh_;
};

class BaroUpdate : public Update_<BaroModel> {
public:
  BaroUpdate();
  BaroUpdate(double pressure);
  BaroUpdate(double pressure, double qnh);
  double qnh() const { return qnh_; }
  BaroUpdate& qnh(double qnh) { qnh_ = qnh; return *this; }

  using Update_<BaroModel>::operator =;

private:
  double qnh_;
};

namespace traits {
  template <> struct Update<BaroModel> { typedef BaroUpdate type; };
}

extern template class Measurement_<BaroModel>;

class Baro : public Measurement_<BaroModel>, HeightBaroCommon
{
public:
  using Measurement_<BaroModel>::Model;
  using Measurement_<BaroModel>::Update;

  enum { MeasurementDimension = Measurement_<BaroModel>::MeasurementDimension };
  using Measurement_<BaroModel>::MeasurementVector;
  using Measurement_<BaroModel>::NoiseVariance;

  Baro(const std::string& name = "baro");
  virtual ~Baro() {}

  void setElevation(double elevation) { getModel()->setElevation(elevation); }
  double getElevation() const { return getModel()->getElevation(); }

  void setQnh(double qnh) { getModel()->setQnh(qnh); }
  double getQnh() const { return getModel()->getQnh(); }

  virtual void onReset();
  virtual bool prepareUpdate(State &state, const Update &update);
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_BARO_H
