// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   Pid.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/

#include "xyBackstepping.h"
#include "xyBackstepping_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

xyBackstepping::xyBackstepping(const LayoutPosition *position, string name)
    : ControlLaw(position->getLayout(), name) {
  pimpl_ = new xyBackstepping_impl(this, position, name);
  
  SetIsReady(true);
}

xyBackstepping::~xyBackstepping(void) { delete pimpl_; }

void xyBackstepping::UseDefaultPlot(const gui::LayoutPosition *position) {
  pimpl_->UseDefaultPlot(position);
}

void xyBackstepping::Reset(void) {
  pimpl_->i = 0;
  pimpl_->first_update = true;
}

float xyBackstepping::GetIntegral(void) const { return pimpl_->i; }

void xyBackstepping::UpdateFrom(const io_data *data) {
  pimpl_->UpdateFrom(data);
  ProcessUpdate(output);
}

void xyBackstepping::SetValues(float pos, float ref_pos, float vel, float ref_vel) {
  input->SetValue(0, 0, pos);
  input->SetValue(1, 0, ref_pos);
  input->SetValue(2, 0, vel);
  input->SetValue(3, 0, ref_vel);
}

} // end namespace filter
} // end namespace flair
