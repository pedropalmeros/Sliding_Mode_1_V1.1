// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/11/07
//  filename:   zBackstepping.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a zBackstepping
//
//
/*********************************************************************/

#include "zBackstepping.h"
#include "zBackstepping_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

zBackstepping::zBackstepping(const LayoutPosition *position, string name)
    : ControlLaw(position->getLayout(), name) {
  pimpl_ = new zBackstepping_impl(this, position, name);
  
  SetIsReady(true);
}

zBackstepping::~zBackstepping(void) { delete pimpl_; }

void zBackstepping::UseDefaultPlot(const flair::gui::LayoutPosition *position) {
  pimpl_->UseDefaultPlot(position);
}

void zBackstepping::Reset(void) {
  ResetI();
  SetDefaultOffset();
  SetValues(0,0,0,0);
}

void zBackstepping::ResetI(void) { pimpl_->i = 0; }

float zBackstepping::GetOffset(void) const { return pimpl_->offset_g; }

float zBackstepping::GetIntegral(void) const { return pimpl_->i; }

void zBackstepping::UpdateFrom(const io_data *data) {
  pimpl_->UpdateFrom(data);
  ProcessUpdate(output);
}

void zBackstepping::SetValues(float pos, float ref_pos, float vel, float ref_vel) {
  input->SetValue(0, 0, pos);
  input->SetValue(1, 0, ref_pos);
  input->SetValue(2, 0, vel);
  input->SetValue(3, 0, ref_vel);
}

void zBackstepping::SetDefaultOffset(void) { pimpl_->offset_g = pimpl_->offset->Value(); }

void zBackstepping::SetOffset(float value) { pimpl_->offset_g = value; }

bool zBackstepping::OffsetStepUp(void) {
  pimpl_->offset_g += pimpl_->pas_offset->Value();
  if (pimpl_->offset_g > 1) {
    pimpl_->offset_g = 1;
    return false;
  } else {
    return true;
  }
}

bool zBackstepping::OffsetStepDown(void) {
  pimpl_->offset_g -= pimpl_->pas_offset->Value();
  if (pimpl_->offset_g < pimpl_->offset->Value()) {
    pimpl_->offset_g = pimpl_->offset->Value();
    return false;
  } else {
    return true;
  }
}

} // end namespace filter
} // end namespace flair
