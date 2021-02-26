//  created:    2011/05/01
//  filename:   PosApp.h
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#ifndef POSAPP_H
#define POSAPP_H

#include <UavStateMachine.h>
#include <Vector3D.h>
#include <Quaternion.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
        class SpinBox;
        class CheckBox;
        class GroupBox;
    }
    namespace filter {
        class attQuatBkstp;
        class xyBackstepping;
        class zBackstepping;
        class xyPD;
        class attSMC;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class PosApp : public flair::meta::UavStateMachine {
    public:
        PosApp(flair::sensor::TargetController *controller);
        ~PosApp();

    private:

	enum class BehaviourMode_t {
            Default,
            AttitudeCtrl,
            AlitutudeMode,
            PositionCtrl
        };


        BehaviourMode_t behaviourMode;


        bool vrpnLost;

        //void ExtraSecurityCheck(void);
        void ExtraCheckPushButton(void);
        void ExtraCheckJoystick(void);


        const flair::core::AhrsData *GetOrientation(void) const;
        //This is used because the z axis is not the same (from inertial to the body frame)
        void AltitudeValues(float &z,float &dz) const;

        
        // Extract the desired attitude. 
        flair::core::AhrsData *GetReferenceOrientation(void);
        void SignalEvent(Event_t event);

        /***********************************************************************
        ***        Dualshock with our attitude controller                    ***
        ************************************************************************
        We need a flags 
        1.- Enable the GetReferenceOrientation measurements from the Dualshok
        2.- Disable the GetReferenceOrientation measurements from the PositionControlLaw.
        ************************************************************************/
        void OnlyAttitudeJoystick_mode(void);


        /**********************************************************************************************
        ***  virtual void flair::meta::UavStateMachine::ComputeCustomTorques(flair::core::Euler&)   ***
        ***********************************************************************************************
         If you want to read the signal from the joystick and set your attitude controller you need
         to override this function, without it, you won't be able to set the tores to your custom

        This funciton accepts a torque object of type Euler, but it works with references, hence in this
        method you will fill the torques object with the attitute control
        torques.roll = roll_ctrl_output
        torques.pithc = pitch_ctrl_output
        torques.yaw = pithch_ctrl_output
         **********************************************************************************************/
        void ComputeCustomTorques(flair::core::Euler &torques);

        float ComputeCustomThrust(void);

        void AltitudeMode(void);

        void xyControl(void);

        void ComputePositionControllers(flair::core::Vector3Df &Controller);

        void ComputeDesiredQuaternion(flair::core::Quaternion &DesiredQuaternion);

        float DotProduct(flair::core::Vector3Df v1, flair::core::Vector3Df v2);


        void GetReferenceQuaternion(flair::core::Quaternion &refQuat, flair::core::Vector3Df &refOmega);

        void GetUAVOrientation(flair::core::Quaternion &uavQuaternion, flair::core::Vector3Df &uavAngSpeed);
 


        flair::core::Vector3Df VectorialProduct(flair::core::Vector3Df m, flair::core::Vector3Df v);



        //This is use to set functions in the QtGUI
        flair::gui::PushButton *attitudePB, *positionPB;
        flair::gui::PushButton *zCtrlPB;


        flair::meta::MetaVrpnObject *uavVrpn;

        //customReferenceOrientation - is the attitude reference from the position control
        //customOrientation - is the attitude (states) of the drone
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;

        flair::gui::Tab *myTab; 
        flair::gui::Tab *setup_myTab, *graph_law_angles, *graph_law_position;

        /////////// CLASS FOR ATTITUDE CONTROL IN QUATERNION and Backstepping
        flair::filter::attQuatBkstp *attQuat;

        //////////// POINTER TO A CLASS FOR THE BACKSTEPPING CONTROL ALONG THE X AXIS///////////
        flair::filter::xyBackstepping *xCtrl;
        flair::gui::DoubleSpinBox *x_des, *y_des, *z_des;



        ////////////    POINTER TO A CLASS FOR THE BACKSTEPPING CONTRO ALONG THE Y AXIS /////////////
        flair::filter::xyBackstepping *yCtrl;

        ////////////    POINTER TO A CLASS FOR THE BACKSTEPPING CONTROL ALONG THE Z AXIS /////////////
        flair::filter::zBackstepping *z_Ctrl;


        ////////////    POINTER TO A CLASS FOR THE SLIDING MODE CONTROL IN ATTITUDE     //////////////
        flair::filter::attSMC   *attSliding;

        ////////////    TENSOR MATRIX
        float Jx = 0.006, Jy = 0.00623,  Jz = 0.1;

        const float mass = 1.2;        

};

#endif // PosApp_H
