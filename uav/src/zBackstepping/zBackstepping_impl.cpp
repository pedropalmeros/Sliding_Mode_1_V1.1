// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/11/07
//  filename:   zBackstepping_impl.cpp
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
#include "zBackstepping_impl.h"
#include "zBackstepping.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream> 

using std::cout;
using std::endl;
using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

zBackstepping_impl::zBackstepping_impl(zBackstepping *self, const LayoutPosition *position,
                               string name) {
  i = 0;
  offset_g = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 4, 1, floatType, name);

  MatrixDescriptor *desc = new MatrixDescriptor(7, 1);
  desc->SetElementName(0, 0, "e1");
  desc->SetElementName(1, 0, "ctrl2virtual");
  desc->SetElementName(2, 0, "e2");
  desc->SetElementName(3, 0, "u+i+d");
  desc->SetElementName(4, 0, "u+i+d+offset");
  desc->SetElementName(5, 0, "z_x4");
  desc->SetElementName(6, 0, "z_d");
  state = new Matrix(self, desc, floatType, name);
  delete desc;

    self->AddDataToLog(state);


  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",0, 1, 0.01);

  kp = new DoubleSpinBox(reglages_groupbox->NewRow(), "kp:", -90000000, 90000000, 0.01);
  kd = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha_2:", -90000000, 90000000, 0.01);
  sat_u = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat_u:", -90000000, 90000000, 0.01);
  _gain = new DoubleSpinBox(reglages_groupbox->NewRow(), "Gain: ", -90000000,90000000,0.001);
  offset = new DoubleSpinBox(reglages_groupbox->NewRow(), "Offset: ",-90000000,90000000,0.001);

}

zBackstepping_impl::~zBackstepping_impl(void) {}

void zBackstepping_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0), DataPlot::Black);
  plot->AddCurve(state->Element(1), DataPlot::Red);

}

void zBackstepping_impl::UpdateFrom(const io_data *data) {
  float p, d, total;
  float delta_t;
  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }



  input->GetMutex();
  float position = input->ValueNoMutex(0,0);
  float ref_position = input->ValueNoMutex(1,0);
  float velocity = input->ValueNoMutex(2,0);
  float ref_velocity = input->ValueNoMutex(3,0);

  //cout << "z: " << position << "   z_d: " << ref_position; 

  float e1= position - ref_position;
  float ctrl2virtual = -kp->Value()*e1 + ref_velocity;
  float e2 = velocity -  ctrl2virtual;
  float u = -kp->Value()*(e2-kp->Value()*e1) - kd->Value()*e2 - e1;


  //float e2 = velocity;

  //float u = -alpha_1->Value()*e1 - alpha_2->Value()*e2;

  input->ReleaseMutex();
  total = u*_gain->Value() + offset->Value();

  //cout << "         total: " << total << endl;




  state->GetMutex();
  state->SetValueNoMutex(0, 0, position);
  state->SetValueNoMutex(1, 0, ref_position);
  state->SetValueNoMutex(2, 0, position);
  state->SetValueNoMutex(3, 0, position);
  state->SetValueNoMutex(4, 0, position);
  state->SetValueNoMutex(5, 0, position);
  state->SetValueNoMutex(6, 0, position);
  state->ReleaseMutex();

  //-offset_g, car on met -u_z dans le multiplex
  // a revoir!
  self->output->SetValue(0, 0, total);
  self->output->SetDataTime(data->DataTime());
}


void zBackstepping_impl::OwnSat(float &signal, float saturation_value){
  if (signal>saturation_value)
    signal = saturation_value;
  if(signal<-saturation_value)
    signal = -saturation_value;
}
