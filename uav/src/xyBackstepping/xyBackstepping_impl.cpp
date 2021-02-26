// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   xyBackstepping_impl.cpp
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
#include "xyBackstepping_impl.h"
#include "xyBackstepping.h"
#include <Matrix.h>
#include <LayoutPosition.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream>
#include <AhrsData.h>
#include <DataPlot1D.h>
#include <math.h>
#include <IODevice.h>



using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

xyBackstepping_impl::xyBackstepping_impl(xyBackstepping *self, const LayoutPosition *position, string name) {
  i = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 4, 1, floatType, name);
  MatrixDescriptor *desc = new MatrixDescriptor(6, 1);
  desc->SetElementName(0, 0, "e1");
  desc->SetElementName(1, 0, "ctrl2virtual");
  desc->SetElementName(2, 0, "e2");
  desc->SetElementName(3, 0, "Ctrl");
  desc->SetElementName(4, 0, "pos");
  desc->SetElementName(5, 0, "ref_pos");
  state = new Matrix(self, desc, floatType, name);
  
  self->AddDataToLog(state);

  delete desc;

  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",0, 1, 0.01);
  alpha_1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_1:", 0, 90000000, 0.01);
  //ki = new DoubleSpinBox(reglages_groupbox->NewRow(), "ki:", 0, 90000000, 0.01,3);
  //sati = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat i:", 0, 1,0.01);
  alpha_2 = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_2:", 0, 90000000, 0.01);
  sat_u = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_u:", 0, 1, 0.1);
  gain_ = new DoubleSpinBox(reglages_groupbox->NewRow(), "gain:",-1,1,0.001);

}

xyBackstepping_impl::~xyBackstepping_impl(void) {}

void xyBackstepping_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0));
  plot->AddCurve(state->Element(1), DataPlot::Green);
  plot->AddCurve(state->Element(2), DataPlot::Blue);
  plot->AddCurve(state->Element(3), DataPlot::Black);
}

void xyBackstepping_impl::UpdateFrom(const io_data *data) {

  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

/****
  if (T->Value() == 0) {
    delta_t = (float)(data->DataDeltaTime() ) / 1000000000.;
  } else {
    delta_t = T->Value();
  }
  if (first_update == true) {
    delta_t = 0;
    first_update = false;
  }
****/
  //cout << "           BacksteppingPosCtrl" << endl; 
  input->GetMutex();
  float position = input->ValueNoMutex(0,0);
  float ref_position = input->ValueNoMutex(1,0);
  float velocity = input->ValueNoMutex(2,0);
  float ref_velocity = input->ValueNoMutex(3,0);

  float e1 = position - ref_position;

  float ctrl2virtual = -alpha_1->Value()*e1 + ref_velocity;

  float e2 = velocity - ctrl2virtual;

  float u = - alpha_1->Value()*(e2 - alpha_1->Value()*e1) - alpha_2->Value()*e2  - e1;

  input->ReleaseMutex();

  u = gain_->Value()*OwnSat(u,sat_u->Value());

  state->GetMutex();
  state->SetValueNoMutex(0, 0, e1);
  state->SetValueNoMutex(1, 0, ctrl2virtual);
  state->SetValueNoMutex(2, 0, e2);
  state->SetValueNoMutex(3, 0, u);
  state->SetValueNoMutex(4, 0, position);
  state->SetValueNoMutex(5, 0, ref_position);

  state->ReleaseMutex();

  self->output->SetValue(0, 0, u);
  self->output->SetDataTime(data->DataTime());
}


float xyBackstepping_impl::OwnSat(float signal, float saturation_value){
  if (signal > saturation_value){
    return saturation_value;
  }
  else if(signal < -saturation_value){
    return -saturation_value;
  }
  else
    return signal;
}