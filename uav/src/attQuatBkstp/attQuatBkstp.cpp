// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   Bkstp.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a Bkstp
//
//
/*********************************************************************/

#include "attQuatBkstp.h"
#include "attQuatBkstp_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

attQuatBkstp::attQuatBkstp(const LayoutPosition *position, string name)
    : ControlLaw(position->getLayout(), name,3) {
  pimpl_ = new attQuatBkstp_impl(this, position, name);
  
  SetIsReady(true);
}

attQuatBkstp::~attQuatBkstp(void) { delete pimpl_; }

void attQuatBkstp::UseDefaultPlot(const gui::LayoutPosition *position) {
  pimpl_->UseDefaultPlot(position);
}

void attQuatBkstp::Reset(void) {
  pimpl_->i = 0;
  pimpl_->first_update = true;
}

float attQuatBkstp::GetIntegral(void) const { return pimpl_->i; }

void attQuatBkstp::UpdateFrom(const io_data *data) {
  pimpl_->UpdateFrom(data);
  ProcessUpdate(output);
}


void attQuatBkstp::SetValues(float q0,        float q1,        float q2,        float q3,
                           float q0_d,      float q1_d,      float q2_d,      float q3_d,
                           float omega_x,   float omega_y,   float omega_z){

  input->SetValue(0, 0, q0);
  input->SetValue(1, 0, q1);
  input->SetValue(2, 0, q2);
  input->SetValue(3, 0, q3);
  input->SetValue(4, 0, q0_d);
  input->SetValue(5, 0, q1_d);
  input->SetValue(6, 0, q2_d);
  input->SetValue(7, 0, q3_d);
  input->SetValue(8, 0, omega_x);
  input->SetValue(9, 0, omega_y);
  input->SetValue(10, 0, omega_z);


}

} // end namespace filter
} // end namespace flair
