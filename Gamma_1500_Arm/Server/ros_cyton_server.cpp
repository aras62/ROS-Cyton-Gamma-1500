//------------------------------------------------------------------------------
// Copyright (c) 2004-2013 Energid Technologies. All rights reserved.
//
/// @file ros_cyton_server.cpp
//
//------------------------------------------------------------------------------


///ROS Headers
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"


///Energid Headers
#include "ecCytonCommands.h"
#include <foundCore/ecApplication.h>
#include <foundCore/ecMacros.h>
#include <foundCommon/ecCoordSysXForm.h>
#include <iostream>
#include <vector>
#include <remoteCommand/ecRemoteCommand.h>

#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
ros::Publisher ee_position_pub;
ros::Publisher joint_values_pub;
ros::Publisher gripper_status_pub;
namespace bpo = boost::program_options;
using namespace std;



//------------------------------------------------------------------------------
#define RC_CHECK(fun) do \
   { \
      std::cout << "Calling " << #fun << std::endl; \
      if(!fun) \
      { \
         std::cerr << "Problem with command " << #fun << "\n"; \
      } \
      else \
      {\
        std::cout << #fun << " successfully completed!" << "\n"; \
      }\
   } while(0)


//-------------------------------------------------------------------------------

EcCytonCommands cytonCommands;

///Global variables for mode ,endeffector, ee_pose,joint_values,gripper value and cyton version
std::string mode;
std::string end_effector_type;
std::vector<double> ee_pose(6); 

std::vector<double> joint_values(7);
double gripper_value = 0.;
EcString cytonVersion;
//saflasf



//-----------------------------------------------------------------------
///Function will send the joint values to actin API's
///@param[in] joints(vector<double>) Joint values of arm in radians,
//@return[out] Status (bool) Return the status of function

bool Send_Joint_Values(vector<double> joints)

{

     ROS_INFO("Inside joint function");
     EcRealVector jointposition(joints);
     jointposition.resize(7);
     RC_CHECK(cytonCommands.MoveJointsExample(jointposition, .000001));//Joint Movement Example
     
     return true;

}


//----------------------------------------------------------------------
///Function will send the end_effector pose  
///@param[in] ee_pose(vector<double>) End effector pose
///@return[out] Status (bool) Status of function

bool Send_EE_Pose(vector<double> ee_pose)

{


  if(end_effector_type == "point_end_effector")
	{      
      		EcCoordinateSystemTransformation desiredPose;
      		desiredPose.setTranslation(EcVector(ee_pose[0],ee_pose[1],ee_pose[2]));
      		RC_CHECK(cytonCommands.pointMovementExample(desiredPose));
	}
  else if(end_effector_type == "frame_end_effector")
	{
	if(cytonVersion=="1500" || cytonVersion=="1500R2")
      		{
         		EcCoordinateSystemTransformation desiredPose;
         		desiredPose.setTranslation(EcVector(ee_pose[0],ee_pose[1],ee_pose[2]));
         		EcOrientation orient;
         		orient.setFrom123Euler(ee_pose[3],ee_pose[4],ee_pose[5]);//set roll, pitch,yaw
         		desiredPose.setOrientation(orient);
         		RC_CHECK(cytonCommands.frameMovementExample(desiredPose));
		}


	}
	return true;

}


//-------------------------------------------------------------------
///Function will send the gripper value
///@param[in] value (double) Value of gripper in meters
///@return[out] Status (bool) Returns the status of the function

bool Send_Gripper_Value(double value)

{
   cytonCommands.moveGripperExample(value);
}


///Callback  for setting the mode
void mode_Callback(const std_msgs::String::ConstPtr& msg)
{

	mode = msg->data;
	ROS_INFO("Mode : [%s]", mode.c_str());
}


///Callback for setting endeffector
void ee_Callback(const std_msgs::String::ConstPtr& msg)
{

	end_effector_type = msg->data; 
	ROS_INFO("End_Effector: [%s]", end_effector_type.c_str()); 

}


///Callback for setting endeffector pose
void ee_pose_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	ee_pose.clear();
	ee_pose = msg->data;
	ROS_INFO("X: [%f]", ee_pose[0]);
	ROS_INFO("Y: [%f]", ee_pose[1]);
	ROS_INFO("Z: [%f]", ee_pose[2]);
	ROS_INFO("ROLL: [%f]", ee_pose[3]);
	ROS_INFO("PITCH: [%f]", ee_pose[4]);
	ROS_INFO("YAW: [%f]", ee_pose[5]);

}

///Callback for setting jointvalues
void joint_value_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

	joint_values.clear();
	joint_values = msg->data;
	ROS_INFO("Joint 0: [%f]", joint_values[0]);
	ROS_INFO("Joint 1: [%f]", joint_values[1]);
	ROS_INFO("Joint 2: [%f]", joint_values[2]);
	ROS_INFO("Joint 3: [%f]", joint_values[3]);
	ROS_INFO("Joint 4: [%f]", joint_values[4]);
	ROS_INFO("Joint 5: [%f]", joint_values[5]);
	ROS_INFO("Joint 6: [%f]", joint_values[6]);

}

///Callback for setting the gripper value
void gripper_Callback(const std_msgs::Float64::ConstPtr& msg)
{

	gripper_value = msg->data;
	ROS_INFO("Gripper value: [%f]", gripper_value);
	//Send_Gripper_Value(gripper_value);
  
}

///Execute callback for running the joint mode and ee mode
void execute_Callback(const std_msgs::String::ConstPtr& msg)
{
   
	ROS_INFO("Execute Command: [%s]", msg->data.c_str());

	if(msg->data == "yes")
	{
		if(mode  == "joint_mode")
			{
				ROS_INFO("Joint Mode");
				Send_Joint_Values(joint_values);
			
			}
		else if(mode == "ik_mode")
			{
				ROS_INFO("IK Mode");
				Send_EE_Pose(ee_pose);
				
			}

		else if (mode == "gripper_mode")
			{
				ROS_INFO("Gripper Mode");
				Send_Gripper_Value(gripper_value);
			}


	}

	else if(msg->data == "no")
	{
		ROS_INFO("Skipping Execution");
	}
	  

}
void sendEEFeedback(const std_msgs::String::ConstPtr& msg)
{
        std::vector<double> EEPosition = cytonCommands.getEEPose();
	std_msgs::Float64MultiArray eePose;
   	eePose.data = EEPosition;	
 	///Sending messages
   	if(msg->data == "yes")
   	{
   	 ee_position_pub.publish(eePose);
         ROS_INFO("EE Feedback Request Received");
       	}

}
void sendGripperFeedback(const std_msgs::String::ConstPtr& msg)
{
        std::vector<double> GripperStatus = cytonCommands.getGripperStatus();
	std_msgs::Float64MultiArray gStat;
   	gStat.data =  GripperStatus;	
 	///Sending messages
   	if(msg->data == "yes")
   	{
   	 gripper_status_pub.publish(gStat);
         ROS_INFO("Gripper Feedback Request Received");
       	}

}

void sendJointFeedback(const std_msgs::String::ConstPtr& msg)
{
	std::vector<double> JValues;
	Ec::getJointValues(JValues);
	std_msgs::Float64MultiArray jointValues;
   	jointValues.data = JValues;	
	///Sending messages
   	if(msg->data == "yes")
   	{
   	 joint_values_pub.publish(jointValues);
         ROS_INFO("Joints Feedback Request Received");
   	}
	
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ros_cyton_server");
  ros::NodeHandle n;
  bpo::options_description options("Options");
   options.add_options()
      (
      "help,h", "Show this help message"
      )
      (
      "ipaddress,i",
      bpo::value<EcString>()->default_value("127.0.0.1"),
      "IpAddress of the computer to connect to"
      )
      (
      "cytonVersion,c",
      bpo::value<EcString>()->default_value("300"),
      "cyton version"
      )
      ;

    // Parse the command line options
   bpo::parsed_options parsed = bpo::command_line_parser(argc, argv).options(options).run();
   bpo::variables_map vm;
   bpo::store(parsed, vm);
   bpo::notify(vm);

   if (vm.count("help") ||(argc == 1))
   {
      std::cout << options << std::endl;
      /*********************valid options*****************************/
      std::cout<<"Cyton Versions: \n300=cyton gamma 300\n300PX=cyton gamma 300 with PX/AX12 gripper\n1500=cyton gamma 1500\n1500R2=cyton gamma 1500R2\""<<std::endl;

      /*********************valid tests*****************************/
      return 1;
   }

   // get config file
   EcString ipAddress = vm["ipaddress"].as<EcString>();
   cytonVersion = vm["cytonVersion"].as<EcString>();

   EcString cytonDir = Ec::Application::getDataDirectory("cyton");
   if(cytonDir.empty())
   {
      cytonDir = ".";
   }
 
   ///Connect to Actin Server
   cytonCommands.openNetwork(ipAddress);




  ///Subscriber List
  ros::Subscriber mode_ = n.subscribe("mode", 100, mode_Callback);
  ros::Subscriber end_effector_type_ = n.subscribe("end_effector_type", 100, ee_Callback);
  ros::Subscriber ee_pose_ = n.subscribe("ee_pose", 100, ee_pose_Callback);
  ros::Subscriber joint_values_ = n.subscribe("joint_array", 10, joint_value_Callback);
  ros::Subscriber gripper_ = n.subscribe("gripper_value", 10, gripper_Callback);
  ros::Subscriber execute_ = n.subscribe("execute", 10,execute_Callback);
  ros::Subscriber EEFeedback = n.subscribe("requestFeedbackEE",10,sendEEFeedback);
  ros::Subscriber JTFeedback = n.subscribe("jointValuesFeedback",10,sendJointFeedback);
  ros::Subscriber GFeedback = n.subscribe("gripperStatusFeedback",10,sendGripperFeedback);
  //publish values
  ee_position_pub = n.advertise<std_msgs::Float64MultiArray>("ee_feedback", 1);
  joint_values_pub = n.advertise<std_msgs::Float64MultiArray>("joint_feedback", 1);
   gripper_status_pub = n.advertise<std_msgs::Float64MultiArray>("gripper_feedback", 1);
  ros::spin();
// %EndTag(SPIN)%

  cytonCommands.closeNetwork();
  exit(0);
  return 0;
}
// %EndTag(FULLTEXT)%

