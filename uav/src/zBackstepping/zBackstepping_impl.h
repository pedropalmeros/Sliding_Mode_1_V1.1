// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file zBackstepping_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2014/11/07
 * \version 4.0
 */

#ifndef ZBACKSTEPPING_IMPL_H
#define ZBACKSTEPPING_IMPL_H

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
class zBackstepping;
}
}

/*! \class zBackstepping_impl
* \brief Class defining a PID
*/

class zBackstepping_impl {
public:
  zBackstepping_impl(flair::filter::zBackstepping *self,
                 const flair::gui::LayoutPosition *position, std::string name);
  ~zBackstepping_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  void OwnSat(float &signal, float saturation_value);


  float i, offset_g;
  flair::gui::DoubleSpinBox *offset, *pas_offset;


private:
  flair::filter::zBackstepping *self;
  bool first_update;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;

  flair::gui::DoubleSpinBox *alpha_1, *alpha_2, *sat_u, *mg, *_gain;
};

#endif // zBackstepping_impl_H
