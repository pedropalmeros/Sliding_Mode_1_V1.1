// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file attSMC_impl.h
 * \brief Classe permettant le calcul d'un Bkstp
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef ATTSMC_IMPL_H
#define ATTSMC_IMPL_H

#include <Object.h>
#include "Eigen/Dense"

using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::MatrixXf;



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
class attSMC;
}
}

/*! \class attSMC_impl
* \brief Class defining a Bkstp
*/

class attSMC_impl {
public:
  attSMC_impl(flair::filter::attSMC *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~attSMC_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float Saturation(float &signal,float saturation);
  int Sign(const float &signal);
  Vector3f Sign(const Vector3f &vect);
  MatrixXf Skew(const Vector3f &vect);



  float i;
  bool first_update;

private:
  flair::filter::attSMC *self;

  // matrix
  flair::core::Matrix *state,*output;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;

  flair::gui::DoubleSpinBox *kp_roll, *kd_roll, *sat_roll;
  flair::gui::DoubleSpinBox *kp_pitch, *kd_pitch, *sat_pitch;
  flair::gui::DoubleSpinBox *kp_yaw, *kd_yaw, *sat_yaw;

  flair::gui::DoubleSpinBox *alpha_x, *alpha_y, *alpha_z;
  flair::gui::DoubleSpinBox *beta_x, *beta_y, *beta_z;
  flair::gui::DoubleSpinBox *rho_x, *rho_y, *rho_z;

  flair::gui::DoubleSpinBox  *satQ;

  flair::gui::DoubleSpinBox *scale_x,*scale_y,*scale_z;

  // TENSOR MATRIX
  float Jx = 0.006, Jy = 0.00623, Jz = 0.1;

  // Gains for Backstepping controller
  flair::gui::DoubleSpinBox *k2,*k3;
  flair::gui::DoubleSpinBox *l2,*l3;
};

#endif // attSMC_impl_H
