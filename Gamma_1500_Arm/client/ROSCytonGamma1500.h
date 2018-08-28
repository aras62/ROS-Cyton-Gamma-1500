/*
 *      Author: Amir Rasouli
 *      email: aras@eecs.yorku.ca
 *
 *     Client sample code for Cyton Gamma 1500 code
 *
 */


#ifndef ROSCYTONGAMMA1500_H
#define ROSCYTONGAMMA1500_H
#include <sstream>
#include <iostream>
#include <vector>

#ifdef WIN32
#include <conio.h>
#else
#include <ncurses.h>
#endif

#define KEY_UP 72 //y axis
#define KEY_DOWN 80//y axis
#define KEY_LEFT 75 //x axis
#define KEY_RIGHT 77//x axis
#define DEPTH_IN 59 //z axis KEY(;)
#define DEPTH_OUT 47//z axis KEY(/)
#define GRIPPER_OPEN 46 // KEY(>)
#define GRIPPER_CLOSE 44 //KEY(<)
#define SHOULDER_ROLL_L 113//Q
#define SHOULDER_ROLL_R 97//A
#define SHOULDER_PITCH_L 119//W
#define SHOULDER_PITCH_R 115//S
#define SHOULDER_YAW_L 101//E
#define SHOULDER_YAW_R 100//D
#define ELBOW_PITCH_L 114//R
#define ELBOW_PITCH_R 102//F
#define WRIST_YAW_R 116//T
#define WRIST_YAW_L 103//G
#define WRIST_PITCH_R 121//Y
#define WRIST_PITCH_L 104//H
#define WRIST_ROLL_R 117//U
#define WRIST_ROLL_L 106//J
#define EE_MODE 49//1
#define JT_MODE 50//2
#define END 122//Z
#define HELP 108//l
#define LoopRate 10
#define NUM_POSES 3
#define NUM_JOINTS 7

#define ARM_LENGTH 0.55 //meter
#define BASE_LENGTH 0.08
#define MAX_EE_CONSTRAINT 0.52
#define MIN_EE_CONTRAINT 0.19




class RosCytonGamma1500 : public IArmController
{
public:
	RosCytonGamma1500();
        ~RosCytonGamma1500(void);
	bool SendEEPose(std::string end_effector , std::vector<double> ee_pose_array);
	bool SendGripperValue(double value) ;
	bool SendJointPose(std::vector<double> joint_pose_array) ;
	std::vector<double> requestEEFeedback() ;
	std::vector<double> requestJointsFeedback() ;
	std::vector<double> requestGripperFeedback();
	void getEEpose(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void getJTValues(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void getGRStatus(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void armDemoCMD();
   	

private:
	///Global declaration of publishers
	ros::Publisher _mode_pub;
	ros::Publisher _ee_type_pub;
	ros::Publisher _ee_pos_pub;
	ros::Publisher _joint_val_pub;
	ros::Publisher _gripper_val_pub;
	ros::Publisher _execute_pub;
	ros::Publisher _ee_feedback_pub;
	ros::Publisher _joints_feedback_pub;
	ros::Publisher _gr_feedback_pub;
	ros::Subscriber _ee_pose;
	ros::Subscriber _joints_vals;
	ros::Subscriber _gr_stat;
	std::vector<double> _ee_feedback;
	std::vector<double> _joint_feedback;
	std::vector<double> _gr_feedback;
	bool _ee_feedback_ready , _joint_feedback_ready , _gr_feedback_ready ;
	
};

#endif //ROSCYTONGAMMA1500_H

