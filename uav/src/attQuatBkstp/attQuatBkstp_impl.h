// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file attQuatBkstp_impl.h
 * \brief Classe permettant le calcul d'un Bkstp
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef ATTQUATBKSTP_IMPL_H
#define ATTQUATBKSTP_IMPL_H

#include <Object.h>

namespace flair {
namespace core {
class Matrix;
class io_data;
}
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
}
namespace filter {
class attQuatBkstp;
}
}

/*! \class attQuatBkstp_impl
* \brief Class defining a Bkstp
*/

class attQuatBkstp_impl {
public:
  attQuatBkstp_impl(flair::filter::attQuatBkstp *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~attQuatBkstp_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float Saturation(float &signal,float saturation);
  int Sign(const float &signal);

  float i;
  bool first_update;

private:
  flair::filter::attQuatBkstp *self;

  // matrix
  flair::core::Matrix *state,*output;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;

  flair::gui::DoubleSpinBox *kp_roll, *kd_roll, *sat_roll;
  flair::gui::DoubleSpinBox *kp_pitch, *kd_pitch, *sat_pitch;
  flair::gui::DoubleSpinBox *kp_yaw, *kd_yaw, *sat_yaw;

  flair::gui::DoubleSpinBox *kpQ, *kdQ, *satQ;

  // TENSOR MATRIX
  float Jx = 0.006, Jy = 0.00623, Jz = 0.1;

  // Gains for Backstepping controller
  flair::gui::DoubleSpinBox *k2,*k3;
  flair::gui::DoubleSpinBox *l2,*l3;
};

#endif // attQuatBkstp_impl_H
