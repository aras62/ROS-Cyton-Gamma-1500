/*
 *      Author: Amir Rasouli
 *      email: aras@eecs.yorku.ca
 *
 *    This code is a ROS client example for Cyton Gamma 1500 arms.
 *    The functions are based on the original examples published by manufacturer ROBAI (www.robai.com) and are slightly modified to be easily used
 *    as part any application. The server files accompanied by this code are the original ones published by the manufacturer.
 *    The code includes a Command line demo in which the arm can be manipulated using keyboard inputs. In addition functionalities
 *    such as receiving feedback is added
 *
 *    Last updated and tested: June 2014
 */

using namespace ros;

//Setting up limits of the arm
double jointLimit[14] = { -2.58799, 2.58799, -1.8126, 1.8126, -1.8126, 1.8126, -1.8126, 1.8126, -1.8126, 1.8126, -1.8126, 1.8126, -2.58799, 2.58799 };
double gripLimit[2] = { -0.006, 0.006 };//{low,high}
double changeStep[4] = { 0.05, 0.05, 0.05, 0.001 }; // {x,y,z,Gripper}
double changeStepJT[7] = { 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05 }; // {x,y,z,Gripper}

RosCytonGamma1500::RosCytonGamma1500()
{
    _ee_feedback_ready = false, _joint_feedback_ready = false, _gr_feedback_ready = false;
   //Setup nodes
    if (!isRosNetworkOk())
    {
        emit message("RosCytonGamma1500: ROS node is not initialized. Cannot setup publishers and subscribers. Exiting.");
        return;
    }
    _ee_feedback_pub = _node->advertise<std_msgs::String>("requestFeedbackEE", 1);
    _mode_pub = _node->advertise<std_msgs::String>("mode", 1);
    _ee_type_pub = _node->advertise<std_msgs::String>("end_effector_type", 1);
    _ee_pos_pub = _node->advertise<std_msgs::Float64MultiArray>("ee_pose", 1);
    _joint_val_pub = _node->advertise<std_msgs::Float64MultiArray>("joint_array", 1);
    _gripper_val_pub = _node->advertise<std_msgs::Float64>("gripper_value", 1);
    _execute_pub = _node->advertise<std_msgs::String>("execute", 1);
    _joints_feedback_pub = _node->advertise<std_msgs::String>("jointValuesFeedback", 1);
    _gr_feedback_pub = _node->advertise<std_msgs::String>("gripperStatusFeedback", 1);
   
 //receive feedback
    _ee_pose = _node->subscribe("ee_feedback", 10, &RosCytonGamma1500::getEEpose, this);
    _joints_vals = _node->subscribe("joint_feedback", 10, &RosCytonGamma1500::getJTValues, this);
    _gr_stat = _node->subscribe("gripper_feedback", 10, &RosCytonGamma1500::getGRStatus, this);
}
RosCytonGamma1500::~RosCytonGamma1500(void)
{}

//Sets the pose of the end effector
bool RosCytonGamma1500::SendEEPose(std::string end_effector , std::vector<double> ee_pose_array)
{
    printf("****Pose Request****") ;
    printf("x:%f" , ee_pose_array[0]);
    printf("y:%f" , ee_pose_array[1]);
    printf("z:%f \n\n" , ee_pose_array[2]);

    // TODO:: Set up a limit condition for end point Effector. This depends on the set up of the arm e.g. the type of platform it is mounted on
    //	if ()
	//	{
    //		printf("Max range is reached\n");
    //		return false;
    //	}

    std_msgs::String ee_type_msg;
    std_msgs::String mode_msg;
    std_msgs::String execute;

    std_msgs::Float64MultiArray ee_pose;
    ee_type_msg.data = end_effector;
    ee_pose.data = ee_pose_array;
    mode_msg.data = "ik_mode";
    execute.data = "yes";
    if (end_effector == "point_end_effector")
    {
        std::stringstream ss1;
        ss1 << "point_end_effector";
        ee_type_msg.data = ss1.str();
        ee_pose.data = ee_pose_array;
    }
    else if (end_effector == "frame_end_effector")
    {
        std::stringstream ss1;
        ss1 << "frame_end_effector";
        ee_type_msg.data = ss1.str();
        ee_pose.data = ee_pose_array;
    }
    std::stringstream ss;
    ss << "ik_mode";
    mode_msg.data = ss.str();
    std::stringstream ss1;
    ss1 << "yes";
    execute.data = ss1.str();

    ros::Rate loop_rate(LoopRate);
    ///Sending messages
    _mode_pub.publish(mode_msg);
    _ee_type_pub.publish(ee_type_msg);
    _ee_pos_pub.publish(ee_pose);
    _execute_pub.publish(execute);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    return true;
}

//Sets the value of the gripper
bool RosCytonGamma1500::SendGripperValue(double value)
{
    //Checking if the limit for the gripper is reached. If so the value is set to maximum/minimum allowable
    value = (value < gripLimit[0]) ? gripLimit[0] : value;
    value = (value > gripLimit[1]) ? gripLimit[1] : value;
    std_msgs::Float64 gripper_val;
    std_msgs::String mode_msg;
    std_msgs::String execute;
    gripper_val.data = value;
    ros::Rate loop_rate(LoopRate);
    std::stringstream ss;
    ss << "gripper_mode";
    mode_msg.data = ss.str();
    std::stringstream ss1;
    ss1 << "yes";
    execute.data = ss1.str();
    _mode_pub.publish(mode_msg);
    _gripper_val_pub.publish(gripper_val);
    _execute_pub.publish(execute);
    ros::spinOnce();
    loop_rate.sleep();

    return true;
}

//Sets the joints angles
bool RosCytonGamma1500::SendJointPose(std::vector<double> joint_pose_array)
{
    printf ("****Joint Request****");
    printf ( "Joint 0:%f", joint_pose_array[0]);
    printf ( "Joint 1:%f", joint_pose_array[1]);
    printf ( "Joint 2:%f", joint_pose_array[2]);
    printf ( "Joint 3:%f", joint_pose_array[3]);
    printf ( "Joint 4:%f", joint_pose_array[4]);
    printf ( "Joint 5:%f", joint_pose_array[5]);
    printf ( "Joint 6:%f \n\n", joint_pose_array[6]);
    std_msgs::String mode_msg;
    std_msgs::String execute;
    std_msgs::Float64MultiArray joint_pose;
    // check to see whether any of the limits is exceeded. If so the values are adjusted accordingly
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        joint_pose_array[i] = (joint_pose_array[i] < jointLimit[2 * i]) ? jointLimit[2 * i] : joint_pose_array[i];
        joint_pose_array[i] = (joint_pose_array[i] > jointLimit[2 * i + 1]) ? jointLimit[2 * i + 1] : joint_pose_array[i];
    }
    joint_pose.data = joint_pose_array;
    std::stringstream ss;
    ss << "joint_mode";
    mode_msg.data = ss.str();
    std::stringstream ss1;
    ss1 << "yes";
    execute.data = ss1.str();
    ///Sending messages
    ros::Rate loop_rate(LoopRate);
    _mode_pub.publish(mode_msg);
    _joint_val_pub.publish(joint_pose);
    _execute_pub.publish(execute);
    ros::spinOnce();
    loop_rate.sleep();
    return true;
}

// Gets the pose of end effector
std::vector<double>  RosCytonGamma1500::requestEEFeedback()
{
    ros::Rate loop_rate(LoopRate);
    std_msgs::String eeFeedback;
    std::stringstream ss1;
    ss1 << "yes";
    eeFeedback.data = ss1.str();
    _ee_feedback_pub.publish(eeFeedback);
    ros::spinOnce();
    loop_rate.sleep();
    return _ee_feedback;
}
void RosCytonGamma1500::getEEpose(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

    _ee_feedback = msg->data;
    _ee_feedback_ready = true;
    printf( "***************End Effector Pose*************");
    printf("x:%f", _ee_feedback[0]);
    printf("y:%f", _ee_feedback[1]);
    printf("z:%f\n\n", _ee_feedback[2]);
    ros::spinOnce();

}

//Gets the current status of the joints
std::vector<double>  RosCytonGamma1500::requestJointsFeedback()
{
    std_msgs::String jFeedback;
    std::stringstream ss1;
    ss1 << "yes";
    jFeedback.data = ss1.str();
    _joints_feedback_pub.publish(jFeedback);
    ros::spinOnce();
    return _joint_feedback;

}
void RosCytonGamma1500::getJTValues(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    _joint_feedback = msg->data;
    _joint_feedback_ready = true;
    printf ("****Joint values****");
    printf ( "Joint 0:%f", _joint_feedback[0]);
    printf ( "Joint 1:%f", _joint_feedback[1]);
    printf ( "Joint 2:%f", _joint_feedback[2]);
    printf ( "Joint 3:%f", _joint_feedback[3]);
    printf ( "Joint 4:%f", _joint_feedback[4]);
    printf ( "Joint 5:%f", _joint_feedback[5]);
    printf ( "Joint 6:%f \n\n", _joint_feedback[6]);
    ros::spinOnce();
}

// Gets the status of the gripper
std::vector<double>  RosCytonGamma1500::requestGripperFeedback()
{
    ros::Rate loop_rate(LoopRate);
    std_msgs::String grFeedback;
    std::stringstream ss1;
    ss1 << "yes";
    grFeedback.data = ss1.str();
    _gr_feedback_pub.publish(grFeedback);
    ros::spinOnce();
    loop_rate.sleep();
     return _gr_feedback;
}
void RosCytonGamma1500::getGRStatus(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

    _gr_feedback = msg->data;
    _gr_feedback_ready = true;
    printf( "***************Gripper Status*************");
    // printf("Gripped_Value1:%f\n\n",_gr_feedback[0]); //uncomment for 3 finger gripper
    // printf("Gripped_Value2:%f\n\n",_gr_feedback[1]);//uncomment for 3 finger gripper
    printf("Gripped_Value3:%f\n\n",_gr_feedback[2]);
    ros::spinOnce();

}

//The following Function is designed for testing functionality of the arm.
//Using keyboard keys you can change individual joints status or the end effector position

void RosCytonGamma1500::armDemoCMD()
{

#ifndef WIN32
    initscr();
    raw();
    keypad(stdscr, TRUE);
    noecho();
#endif

    bool showMain = true, showEE = true, showJT = true;
    bool gripMove = false;
    bool end = false;
    int act = 0;

   //initialize the cordinates of the arm. Alternatively you can get a feedback from the arm to start with
    std::vector<double> cord = { -0.1572, -0.2681, 0.0686, 0, 0, 0 };
    std::vector<double> joints = { -0.703783, 0.975309, -0.137681, 1.34086, -0.329855, 0.840287, 2.11788 };
    double gripStatus = 0.;
    SendGripperValue(gripStatus);
    while (true)
    {
        if (showMain == true)
        {
           printf("\n\n Movement Mode Select");
           printf( "Press 1 for End Effector (EE) mode");
           printf("Press 2 for Joint (JT) mode");
           printf( "Press L for Help");
           printf("Press Z to end\n");
            showMain = false;
        }
        switch (act = getch())
        {
        case EE_MODE:
            while (true)
            {
                if (showEE == true)
                {
                    printf("\n\n End Effector (EE) mode");
                    printf("x axis Keys LEFT RIGHT");
                    printf("y axis Keys UP DOWN");
                    printf("z axis Keys ; /");
                    printf( "Gripper  Keys < >");
                    printf("Press L for Help");
                    printf("Press Z to end\n");
                    showEE = false;
                }
                switch (act = getch())
                {
                case KEY_UP:
                    cord[1] -= changeStep[1];
                    break;
                case KEY_DOWN:
                    cord[1] += changeStep[1];
                    break;
                case KEY_LEFT:
                    cord[0] += changeStep[0];
                    break;
                case KEY_RIGHT:
                    cord[0] -= changeStep[0];
                    break;
                case DEPTH_IN:
                    cord[2] += changeStep[2];
                    break;
                case DEPTH_OUT:
                    cord[2] -= changeStep[2];
                    break;
                case GRIPPER_OPEN:
                    requestGripperFeedback();
                    gripStatus += changeStep[3];
                    SendGripperValue(gripStatus);
                    gripMove = true;
                    break;
                case GRIPPER_CLOSE:
                    requestGripperFeedback();
                    gripStatus -= changeStep[3];
                    SendGripperValue(gripStatus);
                    gripMove = true;
                    break;
                case HELP:
                    showEE = true;
                    break;
                case END:
                    printf("Exit EE mode\n");
                    end = true;
                    showEE = true;
                    joints = requestJointsFeedback();
                    break;
                }
                if (end)
                {
                    end = false;
                    showMain == true;
                    break;
                }
                if (!gripMove){
                    SendEEPose("point_end_effector", cord);
                   //Ucomment if want to receive feedback after each move
                   //requestJointsFeedback();
                   //requestEEFeedback();
                   //requestGripperFeedback();
                }
                else{
                    gripMove = false;
                }
            }
            break;

        case JT_MODE:
            while (true)
            {
                if (showJT == true)
                {
                    printf("\n\n Joint(JT) mode");
                    printf("Shoulder Roll Keys Q A");
                    printf( "Shoulder Pitch Keys W S");
                    printf("Shoulder Yaw Keys E D");
                    printf( "Elbow Pitch Keys R F");
                    printf("Wrist Yaw Keys T G");
                    printf("Wrist Pitch Keys Y H");
                    printf( "Wrist Roll Keys U J");
                    printf( "Gripper  Keys < >");
                    printf("Press L for Help");
                    printf( "Press Z to end\n");
                    showJT = false;
                }

                switch (act = getch())
                {
                case SHOULDER_ROLL_L:
                    joints[0] += changeStepJT[0];
                    break;
                case SHOULDER_ROLL_R:
                    joints[0] -= changeStepJT[0];
                    break;
                case SHOULDER_PITCH_L:
                    joints[1] += changeStepJT[1];
                    break;
                case SHOULDER_PITCH_R:
                    joints[1] -= changeStepJT[1];
                    break;
                case SHOULDER_YAW_L:
                    joints[2] += changeStepJT[2];
                    break;
                case SHOULDER_YAW_R:
                    joints[2] -= changeStepJT[2];
                    break;
                case ELBOW_PITCH_L:
                    joints[3] += changeStepJT[3];
                    break;
                case ELBOW_PITCH_R:
                    joints[3] -= changeStepJT[3];
                    break;
                case WRIST_YAW_R:
                    joints[4] += changeStepJT[4];
                    break;
                case WRIST_YAW_L:
                    joints[4] -= changeStepJT[4];
                    break;
                case WRIST_PITCH_R:
                    joints[5] += changeStepJT[5];
                    break;
                case WRIST_PITCH_L:
                    joints[5] -= changeStepJT[5];
                    break;
                case WRIST_ROLL_R:
                    joints[6] += changeStepJT[6];
                    break;
                case WRIST_ROLL_L:
                    joints[6] -= changeStepJT[6];
                    break;
                case GRIPPER_OPEN:
                    gripStatus += changeStep[3];
                    SendGripperValue(gripStatus);
                    gripMove = true;
                    break;
                case GRIPPER_CLOSE:
                    gripStatus -= changeStep[3];
                    SendGripperValue(gripStatus);
                    gripMove = true;
                    break;
                case HELP:
                    showJT = true;
                    break;
                case END:
                    printf( "Exit JT mode");
                    end = true;
                    showJT = true;
                    break;
                }
                if (end)
                {
                    end = false;
                    showMain == true;
                    break;
                }
                if (!gripMove){
                    SendJointPose(joints);
                    requestJointsFeedback();
                    requestEEFeedback();
                    requestGripperFeedback();
                }
                else{
                    gripMove = false;
                }
            }
            break;
        case HELP:
            showMain = true;
            break;
        case END:
            end = true;
            break;
        default:
            break;
        }
        if (end)
        {
            end = false;

            break;
        }
    }
}
