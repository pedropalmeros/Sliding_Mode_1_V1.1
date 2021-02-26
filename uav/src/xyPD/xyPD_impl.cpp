// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   xyPD_impl.cpp
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
#include "xyPD_impl.h"
#include "xyPD.h"
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

xyPD_impl::xyPD_impl(xyPD *self, const LayoutPosition *position, string name) {
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
  kp = new DoubleSpinBox(reglages_groupbox->NewRow(), "kp:", -100, 90000000, 0.001);
  kd = new DoubleSpinBox(reglages_groupbox->NewRow(), "kd:", -100, 90000000, 0.001);
  sat_u = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_u:", 0, 1, 0.1);
  gain_ = new DoubleSpinBox(reglages_groupbox->NewRow(), "gain:",-1,1,0.001);

}

xyPD_impl::~xyPD_impl(void) {}

void xyPD_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0));
  plot->AddCurve(state->Element(1), DataPlot::Green);
  plot->AddCurve(state->Element(2), DataPlot::Blue);
  plot->AddCurve(state->Element(3), DataPlot::Black);
}

void xyPD_impl::UpdateFrom(const io_data *data) {

  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }


  cout << "PosPD" << endl; 
  input->GetMutex();
  float position = input->ValueNoMutex(0,0);
  float ref_position = input->ValueNoMutex(1,0);
  float velocity = input->ValueNoMutex(2,0);
  float ref_velocity = input->ValueNoMutex(3,0);

  float e1 = position - ref_position;
  float prop_term = kp->Value()*e1;
  float dot_e = velocity - ref_velocity;
  float der_term = kd->Value()*dot_e;

  float u = prop_term + der_term;
  input->ReleaseMutex();

  u = gain_->Value()*OwnSat(u,sat_u->Value());

  state->GetMutex();
  state->SetValueNoMutex(0, 0, e1);
  state->SetValueNoMutex(1, 0, dot_e);
  state->SetValueNoMutex(2, 0, u);
  state->SetValueNoMutex(3, 0, position);
  state->SetValueNoMutex(4, 0, position);
  state->SetValueNoMutex(5, 0, ref_position);

  state->ReleaseMutex();

  self->output->SetValue(0, 0, u);
  self->output->SetDataTime(data->DataTime());
}


float xyPD_impl::OwnSat(float signal, float saturation_value){
  if (signal > saturation_value){
    return saturation_value;
  }
  else if(signal < -saturation_value){
    return -saturation_value;
  }
  else
    return signal;
}