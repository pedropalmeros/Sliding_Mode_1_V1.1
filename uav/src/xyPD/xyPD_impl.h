// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file xyPD_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef xyPD_IMPL_H
#define xyPD_IMPL_H

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
class xyPD;
}
}

/*! \class xyPD_impl
* \brief Class defining a PID
*/

class xyPD_impl {
public:
  xyPD_impl(flair::filter::xyPD *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~xyPD_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float i;
  bool first_update;
  float OwnSat(float signal, float saturation_value);

private:
  flair::filter::xyPD *self;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;
  flair::gui::DoubleSpinBox *alpha_1, *alpha_2;
  flair::gui::DoubleSpinBox *sat_u,*gain_;
};

#endif // xyPD_impl_H
