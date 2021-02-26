// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   attSMC_impl.cpp
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
#include "attSMC_impl.h"
#include "attSMC.h"
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

attSMC_impl::attSMC_impl(attSMC *self, const LayoutPosition *position, string name) {
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

  alpha_x=new DoubleSpinBox(reglages_groupbox->NewRow(),"alpha_x:",-1000,1000,100);
  beta_x=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"beta_x:",-1000,1000,100);
  rho_x=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"rho_x:",-1000,1000,100);
  scale_x = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Scale_x:",-1,1,0.1);



  alpha_y=new DoubleSpinBox(reglages_groupbox->NewRow(),"alpha_y:",-1000,1000,100);
  beta_y=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"beta_y:",-1000,1000,100);
  rho_y=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"rho_y:",-1000,1000,100);
  scale_y = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Scale_y:",-1,1,0.1);

  alpha_z=new DoubleSpinBox(reglages_groupbox->NewRow(),"alpha_z:",-1000,1000,100);
  beta_z=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"beta_z:",-1000,1000,100);
  rho_z=new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"rho_z:",-1000,1000,100);
  scale_z = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Scale_z:",-1,1,0.1);


  satQ  = new DoubleSpinBox(reglages_groupbox->NewRow(), "satQ:", 0, 1, 0.1);

  self->AddDataToLog(state);


}

attSMC_impl::~attSMC_impl(void) {}

void attSMC_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(12), DataPlot::Black);
  plot->AddCurve(state->Element(13), DataPlot::Black);
  plot->AddCurve(state->Element(14), DataPlot::Black);
  plot->AddCurve(state->Element(15), DataPlot::Red);
  plot->AddCurve(state->Element(16), DataPlot::Red);
  plot->AddCurve(state->Element(17), DataPlot::Red);
}

void attSMC_impl::UpdateFrom(const io_data *data) {
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
  Vector3f q_e_vec = q_e.vec();

  //cout << "Error Quaternion ok" << endl; 
  q_e.normalize();


  //  COMPUTING OMEGA_E 
  Vector3f Omega_d_vec(0,0,0);
  Vector3f Omega_vec = Omega.vec();
  Vector3f Omega_e_vec = Omega_vec - Omega_d_vec;

  // COMPUTING DOT_Q_E
  Quaternionf dot_qe;
  dot_qe.w() = -0.5*q_e.vec().dot(Omega_e_vec);
  dot_qe.vec() = 0.5*(q_e.w()*Omega_e_vec+q_e.vec().cross(Omega_e_vec))+q_e.vec().cross(Omega_e_vec);

  // Defining the sliding surface
  MatrixXf beta(3,3);
  beta<< beta_x->Value(),               0,            
     0,
                      0, beta_y->Value(),               0,
                      0,               0, beta_z->Value();

  MatrixXf alpha(3,3);
  alpha<< alpha_x->Value(),               0,            
     0,
                      0, alpha_y->Value(),               0,
                      0,               0, alpha_z->Value();

  //Vector3f beta_qe(beta_x->Value()*q_e.x(),beta_y->Value()*q_e.y(),beta_z->Value()*q_e.z());
  //Vector3f Sigma = Omega_e_vec + beta*q_e.vec();
  Vector3f Sigma = alpha*Omega_e_vec + beta*q_e.vec();

  // DEFINING THE INERTIA MATRXI
  MatrixXf J(3,3);
        J<< Jx, 0, 0,
            0, Jy, 0,
            0, 0, Jz;

  // COMPUTING THE CONTROL LAW
  MatrixXf rho(3,3);
  rho<< rho_x->Value(),              0,              0,
                     0, rho_y->Value(),              0,
                     0,              0, rho_z->Value();

  Vector3f SMC = -rho*Sign(Sigma);

  Vector3f FeedBackLinearization = alpha*Skew(Omega_vec)*J*Omega_vec-J*beta*dot_qe.vec();

  //Vector3f FeedBackLinearization = Skew(Omega_vec)*J*Omega_vec-J*beta*dot_qe.vec();

  //Vector3f Ctrl_vec = FeedBackLinearization + SMC;

  Vector3f Ctrl_vec = alpha.inverse()*(FeedBackLinearization + SMC);


  Ctrl_vec.x() = scale_x->Value()*Ctrl_vec.x();
  Ctrl_vec.y() = scale_y->Value()*Ctrl_vec.y();
  Ctrl_vec.z() = scale_z->Value()*Ctrl_vec.z();

  Ctrl_vec.x() = Saturation(Ctrl_vec.x(),satQ->Value());
  Ctrl_vec.y() = Saturation(Ctrl_vec.y(),satQ->Value());
  Ctrl_vec.z() = Saturation(Ctrl_vec.z(),satQ->Value());

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

  self->output->SetValue(0, 0, Ctrl_vec.x());
  self->output->SetValue(1, 0, Ctrl_vec.y());
  self->output->SetValue(2, 0, Ctrl_vec.z());
  self->output->SetDataTime(data->DataTime());
}

float attSMC_impl::Saturation(float &signal, float saturation){
  if (signal>=saturation)
    return saturation;
  else if (signal <= -saturation)
    return -saturation;
  else
    return signal;
}


 int attSMC_impl::Sign(const float &signal){
  if(signal>=0)
    return 1;
  else
    return -1;
}

Vector3f attSMC_impl::Sign(const Vector3f &vect){
  return Vector3f(tanh(vect.x()),tanh(vect.y()),tanh(vect.z()));
}


MatrixXf attSMC_impl::Skew(const Vector3f &vect){
  MatrixXf S(3,3);
  S<< 0        , -vect.z(),  vect.y(),
       vect.z(),  0       , -vect.x(),
      -vect.y(),  vect.x(),  0;
  return S;

}