// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   attQuatBkstp_impl.cpp
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
#include "attQuatBkstp_impl.h"
#include "attQuatBkstp.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream>
#include <math.h>
#include "Eigen/Dense"

#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <GroupBox.h>
#include <Quaternion.h>
#include <AhrsData.h>
#include <DataPlot1D.h>
#include <math.h>
#include <Euler.h>

#include <IODevice.h>


using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::ArrayXf;
using Eigen::Quaternion;
using Eigen::Quaternionf;

attQuatBkstp_impl::attQuatBkstp_impl(attQuatBkstp *self, const LayoutPosition *position, string name) {
  i = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 11, 1, floatType, name);

  MatrixDescriptor* desc = new MatrixDescriptor(18, 1);
  desc->SetElementName(0, 0, "qe0");
  desc->SetElementName(1, 0, "qe1");
  desc->SetElementName(2, 0, "qe2");
  desc->SetElementName(3, 0, "qe3");
  desc->SetElementName(4, 0, "Qquad.q0");
  desc->SetElementName(5, 0, "Qquad.q1");
  desc->SetElementName(6, 0, "Qquad.q2");
  desc->SetElementName(7, 0, "Qquad.q3");
  desc->SetElementName(8, 0, "Q_d.q0");
  desc->SetElementName(9, 0, "Q_d.q1");
  desc->SetElementName(10, 0, "Q_d.q2");
  desc->SetElementName(11, 0, "Q_d.q3");
  desc->SetElementName(12, 0, "Quad_Euler.roll");
  desc->SetElementName(13, 0, "Quad_Euler.pitch");
  desc->SetElementName(14, 0, "Quad_Euler.yaw");
  desc->SetElementName(15, 0, "Quad_Euler_d.roll");
  desc->SetElementName(16, 0, "Quad_Euler_d.pitch");
  desc->SetElementName(17, 0, "Quad_Euler_d.yaw");  
  state = new flair::core::Matrix(self, desc, floatType, name);

  delete desc;




  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s", 0, 1, 0.01);

  //kpQ  = new DoubleSpinBox(reglages_groupbox->NewRow(), "kpQuat:", 0, 90000000, 0.01,3);
  //kdQ  = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "kdQuat:", 0, 90000000, 0.01,3);
  k2=new DoubleSpinBox(reglages_groupbox->NewRow(),"k2:",-1000,1000,100);
  l2=new DoubleSpinBox(reglages_groupbox->NewRow(),"l2:",-1000,1000,100);
      
  k3=new DoubleSpinBox(reglages_groupbox->NewRow(),"k3:",-1000,1000,100);
  l3=new DoubleSpinBox(reglages_groupbox->NewRow(),"l3:",-1000,1000,100);
  satQ = new DoubleSpinBox(reglages_groupbox->NewRow(), "satQ:", 0, 1, 0.1);

  self->AddDataToLog(state);





}

attQuatBkstp_impl::~attQuatBkstp_impl(void) {}

void attQuatBkstp_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(12), DataPlot::Black);
  plot->AddCurve(state->Element(13), DataPlot::Black);
  plot->AddCurve(state->Element(14), DataPlot::Black);
  plot->AddCurve(state->Element(15), DataPlot::Red);
  plot->AddCurve(state->Element(16), DataPlot::Red);
  plot->AddCurve(state->Element(17), DataPlot::Red);
}

void attQuatBkstp_impl::UpdateFrom(const io_data *data) {
  float p, d, total;
  float delta_t;
  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  if (T->Value() == 0) {
    delta_t = (float)(data->DataDeltaTime() ) / 1000000000.;
  } else {
    delta_t = T->Value();
  }
  if (first_update == true) {
    delta_t = 0;
    first_update = false;
  }

  input->GetMutex();

  // Generating the quaternions to obtain the quaternion error
  Quaternionf q(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
  Quaternionf q_d(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0),input->ValueNoMutex(7,0));
  Quaternionf Omega(0.0,input->ValueNoMutex(8,0), input->ValueNoMutex(9,0), input->ValueNoMutex(10,0));

  input->ReleaseMutex();


  //Once the quaternions have been created they need to be normalied
  q.normalize();
  q_d.normalize();
  flair::core::Quaternion Qquad(q.w(), q.x(), q.y(), q.z());
  flair::core::Quaternion Q_d(q_d.w(),q_d.x(),q_d.y(),q_d.z()); 

  flair::core::Euler Quad_Euler = Qquad.flair::core::Quaternion::ToEuler();
  flair::core::Euler Quad_Euler_d = Q_d.flair::core::Quaternion::ToEuler();



  //Generating the Quaternion error
  Quaternionf q_e = q_d.conjugate()*q;

  //cout << "Error Quaternion ok" << endl; 
  q_e.normalize();

  MatrixXf z_1(4,1);
  z_1(0,0) = 1-abs(q_e.w()); //
  //z_1(0,0) = sgn(errorQuaternion.q0); //
  z_1(1,0) = q_e.x(); // 
  z_1(2,0) = q_e.y(); // 
  z_1(3,0) = q_e.z();

  MatrixXf P(4,3);
  P<< 0.5*(Sign(q_e.w())*q_e.x()),  0.5*(Sign(q_e.w())*q_e.y()), 0.5*(Sign(q_e.w())*q_e.z()),
             0.5*(q.w()),  0.5*(q.z()), -0.5*(q.y()),
            -0.5*(q.z()),  0.5*(q.w()),  0.5*(q.x()),
             0.5*(q.y()), -0.5*(q.x()),  0.5*(q.w());

  if(q_e.w()<0.){
    q_e.w() = -q_e.w();
  }else{
    q_e.w() = q_e.w();
  }

  MatrixXf L(3,3);
  L<< l2->Value(),           0,           0,
                0, l2->Value(),           0,
                0,           0, l3->Value();

  // Computing the first virtual control
  MatrixXf Omega_q_v(3,1);
  Omega_q_v = -L*P.transpose()*z_1;

  MatrixXf Omega_bar = Omega.vec();

  //Computing the second virtual error
  MatrixXf z_2(3,1);
  z_2.setZero();
  z_2 = Omega_bar - Omega_q_v;

  //Compute the control Law
  MatrixXf S(3,3);
  S<< 0        , -Omega.z(),  Omega.y(),
      Omega.z(),  0        , -Omega.x(),
     -Omega.y(),  Omega.x(),  0;

  MatrixXf J(3,3);
        J<< Jx, 0, 0,
            0, Jy, 0,
            0, 0, Jz;

  MatrixXf K(3,3);
  K<< k2->Value(),           0,           0,
      0          , k2->Value(),           0,
      0          ,           0, k3->Value();

  MatrixXf Tau(3,1);
  Tau  = S*J*Omega_bar-K*z_2-P.transpose()*z_1;

  MatrixXf Ctrl_sat = Tau;

  Ctrl_sat(0,0) = Saturation(Ctrl_sat(0,0),satQ->Value());
  Ctrl_sat(1,0) = Saturation(Ctrl_sat(1,0),satQ->Value());
  Ctrl_sat(2,0) = Saturation(Ctrl_sat(2,0),satQ->Value());


  state->GetMutex();
  state->SetValueNoMutex(0, 0, q_e.w());
  state->SetValueNoMutex(1, 0, q_e.x());
  state->SetValueNoMutex(2, 0, q_e.y());
  state->SetValueNoMutex(3, 0, q_e.z());
  state->SetValueNoMutex(4, 0, Qquad.q0);
  state->SetValueNoMutex(5, 0, Qquad.q1);
  state->SetValueNoMutex(6, 0, Qquad.q2);
  state->SetValueNoMutex(7, 0, Qquad.q3);
  state->SetValueNoMutex(8, 0, Q_d.q0);
  state->SetValueNoMutex(9, 0, Q_d.q1);
  state->SetValueNoMutex(10, 0, Q_d.q2);
  state->SetValueNoMutex(11, 0, Q_d.q3);
  state->SetValueNoMutex(12, 0, Quad_Euler.roll);
  state->SetValueNoMutex(13, 0, Quad_Euler.pitch);
  state->SetValueNoMutex(14, 0, Quad_Euler.yaw);
  state->SetValueNoMutex(15, 0, Quad_Euler_d.roll);
  state->SetValueNoMutex(16, 0, Quad_Euler_d.pitch);
  state->SetValueNoMutex(17, 0, Quad_Euler_d.yaw);  

  state->ReleaseMutex();

  self->output->SetValue(0, 0, Ctrl_sat(0,0));
  self->output->SetValue(1, 0, Ctrl_sat(1,0));
  self->output->SetValue(2, 0, Ctrl_sat(2,0));
  self->output->SetDataTime(data->DataTime());
}

float attQuatBkstp_impl::Saturation(float &signal, float saturation){
  if (signal>=saturation)
    return saturation;
  else if (signal <= -saturation)
    return -saturation;
  else
    return signal;
}


 int attQuatBkstp_impl::Sign(const float &signal){
  if(signal>=0)
    return 1;
  else
    return -1;
}