 //------------------------------------------------------------------------------
// Copyright (c) 2004-2013 Energid Technologies. All rights reserved.
//
/// @file ecCytonCommands.cpp
//
//------------------------------------------------------------------------------
#include "ecCytonCommands.h"
#include <control/ecEndEffectorSet.h>
#include <control/ecFrameEndEffector.h>
#include <control/ecManipEndEffectorPlace.h>
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <manipulation/ecManipulationActionManager.h>
#include <math.h>
#include <remoteCommand/ecRemoteCommand.h>
#include <xml/ecXmlObjectReaderWriter.h>
#include <iostream>

#define POINT_EE_SET 0
#define FRAME_EE_SET 1
#define JOINT_CONTROL_EE_SET 0xFFFFFFFF

//------------------------------------------------------------------------------
// Callback function.  Sets a local variable with completion status.
static EcBoolean g_ManipulationComplete = EcFalse;
void manipCallback(EcBoolean status, void* data)
{
   std::cout << "Received sequence completed status of " << (status ? "SUCCESS" : "FAILURE") << std::endl;
   g_ManipulationComplete = EcTrue;
}

using namespace Ec;
//----------------------------------constructor--------------------------------
EcCytonCommands::EcCytonCommands
   (
   )
{
}

//----------------------------------destructor---------------------------------
EcCytonCommands::~EcCytonCommands
   (
   )
{
}

//----------------------------------overloading = -----------------------------
const EcCytonCommands& EcCytonCommands:: operator=
   (
   const EcCytonCommands& orig
   )const
{
   return *this;
}

//----------------------------------overloading == ----------------------------
EcBoolean EcCytonCommands:: operator==
   (
   const EcCytonCommands& orig
   )const
{
   return EcTrue;
}

//----------------------------------open network-------------------------------
EcBoolean EcCytonCommands::openNetwork
   (
   const EcString& m_IpAddress
   )const
{
   EcBoolean retVal = EcTrue;
   if(!init(m_IpAddress))
   {
      std::cerr << "Failed to init\n";
      return EcFalse;
   }
   retVal &= setManipulationCompletedCallback(manipCallback);
   return EcTrue;
}

//----------------------------------close network------------------------------
EcBoolean EcCytonCommands::closeNetwork
   (
   )const
{
   shutdown();
   return EcTrue;
}

//----------------------------------joint commands test------------------------

EcBoolean EcCytonCommands::MoveJointsExample
   (
   const EcRealVector jointPosition,
   const EcReal angletolerance
   )const
{
   EcBoolean retVal=EcTrue;
   setEndEffectorSet(JOINT_CONTROL_EE_SET);
   EcSLEEPMS(500);
   EcRealVector currentJoints;//vector of EcReals that holds the set of joint angles
   retVal &= getJointValues(currentJoints);

   size_t size = currentJoints.size();
   if(size < jointPosition.size())
   {
      size = currentJoints.size();
   }
   else if(size >= jointPosition.size())
   {
      size = jointPosition.size();
   }

   std::cout<<"Current Joint Angles: ( ";
   for(size_t ii=0; ii<size; ++ii)
   {
      std::cout << currentJoints[ii] << "," ;
      currentJoints[ii] = jointPosition[ii];
   }
   std::cout << " )" << std::endl;

   std::cout << "Desired joint Angles: ( ";
   for(size_t ii=0; ii<size; ++ii)
   {
      std::cout << currentJoints[ii] << "," ;
   }
   std::cout <<" )" << std::endl;

   retVal &= setJointValues(currentJoints);

   //Check if achived
   EcBooleanVector jointAchieved;
   jointAchieved.resize(size);
   EcBoolean positionAchieved = EcFalse;
   while(!positionAchieved)
   {
      EcPrint(Debug) << "Moving ";
      getJointValues(currentJoints);
      EcPrint(Debug) << "Current Joints: ";
      for(size_t ii=0; ii<size; ++ii)
      {

         EcPrint(Debug)  << " , " << currentJoints[ii];

         if( abs(jointPosition[ii]-currentJoints[ii])<angletolerance)
         {
            jointAchieved[ii]=EcTrue;
         }
      }
      EcPrint(Debug) <<std::endl;
      for(size_t ii=0; ii<size; ++ii)
      {
         if(!jointAchieved[ii])
         {
            positionAchieved=EcFalse;
            break;
         }
         else
         {
            positionAchieved=EcTrue;
         }
      }  
   }
   std::cout<<" Final Joint Angles: (";
   for(size_t ii=0; ii<size; ++ii)
   {
      std::cout <<  currentJoints[ii] << "," ;
   }
   std::cout<<" ) " << std::endl;

   return retVal;
}

//-----------------------------Point Movement Example-------------------------
EcBoolean EcCytonCommands::pointMovementExample
   (
   const EcCoordinateSystemTransformation& pose
   )const
{

   std::cout<<"Desired pose:  x: "<<pose.translation().x()<< " y: " <<pose.translation().y()<<" z: " <<pose.translation().z()<<std::endl;

   setEndEffectorSet(POINT_EE_SET); // point end effector set index
   EcEndEffectorPlacement desiredPlacement(pose);
   EcManipulatorEndEffectorPlacement actualEEPlacement,desiredEEPlacement;
   EcCoordinateSystemTransformation offset, zero, desiredCoord, actualCoord;
   zero.setTranslation(EcVector(0,0,0));

   getActualPlacement(desiredEEPlacement);
   EcEndEffectorPlacementVector state = desiredEEPlacement.offsetTransformations();
   state[0]=desiredPlacement;
   setDesiredPlacement(desiredPlacement,0,0);

   EcBoolean achieved = EcFalse;
   while(!achieved)
   {
      EcPrint(Debug) << "Moving "<<std::endl;
      getActualPlacement(actualEEPlacement);
      actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
      getDesiredPlacement(desiredEEPlacement);
      desiredCoord=desiredEEPlacement.offsetTransformations()[0].coordSysXForm();

      //get the transformation between the actual and desired 
      offset=(actualCoord.inverse()) * desiredCoord;
      EcPrint(Debug)<<"distance between actual and desired: "<<offset.translation().mag()<<std::endl;

      if(offset.approxEq(zero,.00001))
      {
         std::cout<<"Achieved Pose"<<std::endl;
         achieved = EcTrue;
      }
   }
   return achieved;
}
std::vector<double> EcCytonCommands::getEEPose()
		{
		EcManipulatorEndEffectorPlacement actualEEPlacement;
	 	EcCoordinateSystemTransformation actualCoord;
		getActualPlacement(actualEEPlacement);
		actualCoord = actualEEPlacement.offsetTransformations()[0].coordSysXForm();
		std::vector<double> coord;
		coord.push_back(actualCoord.translation().x());
		coord.push_back(actualCoord.translation().y());
		coord.push_back(actualCoord.translation().z());
		return coord;
		}
 std::vector <double> EcCytonCommands::getGripperStatus()
{
   EcManipulatorEndEffectorPlacement actualEEPlacement;
   //get the current placement of the end effectors
   getActualPlacement(actualEEPlacement);
   //0 is the Wrist roll joint (point or frame end effector), 
   //1 is the first gripper finger (linear constraint end effector)
    EcCoordinateSystemTransformation gripperfinger1trans  = actualEEPlacement.offsetTransformations()[1].coordSysXForm();
		std::vector<double> coord;
		coord.push_back(gripperfinger1trans.translation().x());
		coord.push_back(gripperfinger1trans.translation().y());
		coord.push_back(gripperfinger1trans.translation().z());
 return coord;
}
//-----------------------------Frame Movement Example-------------------------
EcBoolean EcCytonCommands::frameMovementExample
   (
   const EcCoordinateSystemTransformation& pose
   )const
{

   std::cout<<"Desired pose:  x: "<<pose.translation().x()<< " y: " <<pose.translation().y()<<" z: " <<pose.translation().z()<<std::endl;

   setEndEffectorSet(FRAME_EE_SET); // frame end effector set index
   EcEndEffectorPlacement desiredPlacement(pose);
   EcManipulatorEndEffectorPlacement actualEEPlacement,desiredEEPlacement;
   EcCoordinateSystemTransformation offset, zero, desiredCoord, actualCoord;
   zero.setTranslation(EcVector(0,0,0));

   getActualPlacement(actualEEPlacement);
   EcEndEffectorPlacementVector state = actualEEPlacement.offsetTransformations();
   state[0]=desiredPlacement;
   setDesiredPlacement(desiredPlacement,0,0);

   EcBoolean achieved = EcFalse;
   while(!achieved)
   {
      EcPrint(Debug) << "Moving "<<std::endl;
      getActualPlacement(actualEEPlacement);
      actualCoord=actualEEPlacement.offsetTransformations()[0].coordSysXForm();
      getDesiredPlacement(desiredEEPlacement);
      desiredCoord=desiredEEPlacement.offsetTransformations()[0].coordSysXForm();

      //get the transformation between the actual and desired 
      offset=(actualCoord.inverse()) * desiredCoord;
      EcPrint(Debug)<<"distance between actual and desired: "<<offset.translation().mag()<<std::endl;

      if(offset.approxEq(zero,.00001))
      {
         EcPrint(Debug)<<"Achieved Pose"<<std::endl;
         achieved = EcTrue;
      }
   }
   return achieved;
}

//-----------------------------move gripper test-------------------------
EcBoolean EcCytonCommands::moveGripperExample
   (
   const EcReal gripperPos
   )const
{
 
 EcBoolean retVal=EcTrue;

   EcManipulatorEndEffectorPlacement actualEEPlacement,desiredEEPlacement;

   setEndEffectorSet(FRAME_EE_SET); // frame end effector set index
   EcSLEEPMS(100);
   //get the current placement of the end effectors
   getActualPlacement(actualEEPlacement);

   //0 is the Wrist roll joint (point or frame end effector), 
   //1 is the first gripper finger (linear constraint end effector)
   EcEndEffectorPlacementVector state = actualEEPlacement.offsetTransformations();

   //set the trnaslation of the driving gripper finger
   EcCoordinateSystemTransformation gripperfinger1trans = state[1].coordSysXForm();
   gripperfinger1trans.setTranslation(EcVector(0,0,gripperPos));
   EcEndEffectorPlacement finger1placement = state[1];
   finger1placement.setCoordSysXForm(gripperfinger1trans);
   state[1]=finger1placement;

   desiredEEPlacement.setOffsetTransformations(state);

   //set the desired placement
   setDesiredPlacement(desiredEEPlacement,0);

   return retVal;
}









//-----------------------------manipulation action test-------------------------
EcBoolean EcCytonCommands::manipulationActionTest
   (
   const EcString& filename,
   const EcString& actionName,
   const EcU32& timeInMS
   )const
{
   EcManipulationActionManager actionManager;
   EcBoolean retVal = EcXmlObjectReaderWriter::readFromFile(actionManager, filename);

   if(!retVal)
   {
      return retVal;
   }

   retVal = setManipulationActionManager(actionManager);
   if(!retVal)
   {
      return retVal;
   }

   retVal = setManipulationAction(actionName);
   if(!retVal)
   {
      return retVal;
   }

   retVal = startManipulation();
   if(!retVal)
   {
      return retVal;
   }

   EcSLEEPMS(timeInMS);

   retVal = stopManipulation();
   if(!retVal)
   {
      return retVal;
   }
   return retVal;
}

EcBoolean EcCytonCommands::pickAndPlaceExample
   (
   const EcString& cytonModel
   )const
{
   EcRealVector initJoints(7);//vector of EcReals that holds the set of joint angles
   initJoints[1]=-.7;
   initJoints[3]=-.7;
   initJoints[5]=-.7;

   //Move Joints
   MoveJointsExample(initJoints,.000001);

   //open the gripper
   if(cytonModel == "1500" || cytonModel == "300" )
   {
      moveGripperExample(0.0078);
   }
   if(cytonModel == "1500R2" || cytonModel == "300PX" )
   {
      moveGripperExample(0.0149);
   }


   if(cytonModel == "1500" || cytonModel == "1500R2")
   {

      EcCoordinateSystemTransformation desiredPose;
      desiredPose.setTranslation(EcVector(0,.35,.2));
      EcOrientation orient;
      orient.setFrom123Euler(0,0,EcPi/2);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.115,.35,.2));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.115,.35,.05));
      frameMovementExample(desiredPose); 

      //close the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(-0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.001);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(.115,.35,.2));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(0,.35,.2));
      frameMovementExample(desiredPose);   

      orient.setFrom123Euler(0,0,EcPi);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);
      frameMovementExample(desiredPose); 

      desiredPose.setTranslation(EcVector(0,.35,.05));
      frameMovementExample(desiredPose);   


      //Opem the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.0149);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(0,.35,.2));
      frameMovementExample(desiredPose);   
   }

   if(cytonModel=="300" || cytonModel=="300PX" )
   {
      EcCoordinateSystemTransformation desiredPose;
      desiredPose.setTranslation(EcVector(0,.2,.15));
      EcOrientation orient;
      orient.setFrom123Euler(0,0,EcPi/2);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);

      setEndEffectorSet(FRAME_EE_SET); // frame end effector set index

      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.1,.2,.15));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(.1,.2,.05));
      frameMovementExample(desiredPose); 

      //close the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(-0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.001);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(.1,.2,.15));
      frameMovementExample(desiredPose);   

      desiredPose.setTranslation(EcVector(0,.2,.15));
      frameMovementExample(desiredPose);   

      orient.setFrom123Euler(0,0,EcPi);//set roll, pitch,yaw
      desiredPose.setOrientation(orient);
      frameMovementExample(desiredPose); 

      desiredPose.setTranslation(EcVector(0,.2,.05));
      frameMovementExample(desiredPose);   


      //close the gripper
      if(cytonModel == "1500" || cytonModel == "300" )
      {
         moveGripperExample(0.0078);
      }
      if(cytonModel == "1500R2" || cytonModel == "300PX" )
      {
         moveGripperExample(0.0149);
      }
      EcSLEEPMS(1000);

      desiredPose.setTranslation(EcVector(0,.2,.15));
      frameMovementExample(desiredPose);   
   }
   return EcTrue;


}



EcBoolean EcCytonCommands::hardwareEnableTest
   (
   const EcBoolean flag
   )const
{
   return setHardwareEnable(flag);
}

EcBoolean EcCytonCommands::resetToHome
   (
   )const
{
   EcRealVector joints;
   EcBoolean retVal = getJointValues(joints);

   size_t size = joints.size();

   // increment all joints except the last
   for(size_t ii=0; ii<size; ++ii)
   {
      joints[ii] = 0.0;
   }

   retVal &= setJointValues(joints);
   EcSLEEPMS(2000);

   return retVal;
}
