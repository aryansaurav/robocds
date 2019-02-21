#include "robocds.h"
#include "CDSExecution.h"
#include "GMR.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mathlib/MathLib.h"


#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/PointStamped.h"

#include <sstream>




    Eigen::Vector3f _x;      // Current position [m] (3x1)
    Eigen::Vector3f _x0;     // Initial end effector postion (3x1)
    Eigen::Vector3f _v;      // Current end effector velocity [m/s] (3x1)
    Matrix3 _wRb;    // Current rotation matrix (3x3)
    Eigen::Vector4f _q;      // Current end effector quaternion (4x1)
    bool firstrosread = false;


Matrix3 quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Matrix3 R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
	if(firstrosread== false)
	{
		firstrosread = true;
		ROS_INFO("First position read!");
	}


	 geometry_msgs::Pose _msgRealPose = *msg;

	// Update end effecotr pose (position+orientation)
	_x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
	_q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;


}


int main(int argc, char **argv)

{	
	int count =0;


	GMRDynamics master_arm ("data/masterGMM.txt");
	GMRDynamics slave_hand("data/slaveGMM.txt");
//	GMRDynamics slave_fingers("data/?.txt");

	GMR coupling_hand("data/cplGMM.txt");
//	GMR coupling_fingers("... ");

	cds_hand->init(&master_arm, &slave_hand, &coupling_hand);
//	cds_fingers->init(&master_arm, &slave_fingers, &coupling_fingers);

	Vector3 target_trans;
	REALTYPE target_orien[9];

	std::cout<<"\n Enter target translation:(vector 3)"<<endl;
	for(int i=0; i<3; i++)
		std::cin>>target_trans[i];

	std::cout<<	"\n Enter target orientation:(matrix 3)"<<endl;
	for(int i=0; i<9; i++)
		std::cin>>target_orien[i];


	Matrix4 attractor_frame(1,0,0,0,
						0,1,0,0,
						0,0,1,0,
						0,0,0,0);
	//Matrix4 attractor_frame(1,0,0,0.5,
	//					0,1,0,0,
	//					0,0,1,0,
	//					0,0,0,0);

	Matrix4 object_frame(target_orien, target_trans);




	Matrix4 curr_ee_pose, des_ee_pose;
	Vector3 curr_ee_trans;
	Vector des_ee_quaternion, curr_ee_quaternion;
	Matrix3 curr_ee_orient;


	CDSController::DynamicsType master_type;//= MODEL_DYNAMICS;()
	
	CDSController::DynamicsType slave_type ;//= MODEL_DYNAMICS;

	cds_hand->setObjectFrame(object_frame);
	cds_hand->setAttractorFrame(attractor_frame);
	cds_hand->setDT(0.001);
	cds_hand->setMotionParameters(0.5,0.2,1,0.1,static_cast<CDSController::DynamicsType>(1), static_cast<CDSController::DynamicsType>(1));



    geometry_msgs::Pose _msgRealPose, _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;


	ros::init(argc, argv, "input");
	
	ros::NodeHandle n1;

	ros::Publisher desiredtwist = n1.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1000);
    ros::Publisher desiredOrientation = n1.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
   	ros::Publisher desiredPose = n1.advertise<geometry_msgs::Pose>("/lwr/joint_controllers/command_pos", 1000);


//	ros::Subscriber subRealTwist = n1.subscribe("/lwr/ee_vel", 1, updateRealTwist);
	ros::Subscriber subRealPose = n1.subscribe("/lwr/ee_pose", 1, updateRealPose);
	

	ros:: Rate loop_rate(10);

	while(!firstrosread)
	{
			ros::spinOnce();
			loop_rate.sleep();
	}

		_wRb = quaternionToRotationMatrix(_q);
		curr_ee_trans[0] = _x[0];
		curr_ee_trans[1] = _x[1];
		curr_ee_trans[2] = _x[2];
		curr_ee_pose.SetTransformation(_wRb, curr_ee_trans);
//		curr_ee_orient = quaternionToRotationMatrix(_q);


	

	std::cout<<"\nInitial_ee_pose Translation:"<<endl;
	for(int i =0;i<3;i++)
//			cout<<curr_ee_pose.GetTranslation()[i]<< ", ";
			cout<<_x[i]<< ", ";

	curr_ee_pose.GetOrientation().GetQuaternionRepresentation(curr_ee_quaternion);

	std::cout<<"\nInitial_ee_pose Quaternion:"<<endl;
	for(int i =0;i<4;i++)
//			cout<<curr_ee_quaternion[i]<<", ";
			cout<<_q[i]<< ", ";

	cds_hand->setCurrentEEPose(curr_ee_pose);
	cds_hand->postInit();

	

	cout<<"\n Success flag"<<endl;

	while(ros::ok())
	{

		std::cout<<"\nTarget Translation:"<<endl;
	for(int i =0;i<3;i++)
			cout<<object_frame.GetTranslation()[i]<< ", ";

	object_frame.GetOrientation().GetQuaternionRepresentation(curr_ee_quaternion);

	std::cout<<"\nTarget Quaternion:"<<endl;
	for(int i =0;i<4;i++)
			cout<<curr_ee_quaternion[i]<<", ";


		_wRb = quaternionToRotationMatrix(_q);
		curr_ee_trans[0] = _x[0];
		curr_ee_trans[1] = _x[1];
		curr_ee_trans[2] = _x[2];
		curr_ee_pose.SetTransformation(_wRb, curr_ee_trans);



		cds_hand->setCurrentEEPose(curr_ee_pose);
		des_ee_pose= cds_hand->getNextEEPose();

		_msgDesiredTwist.linear.x  = des_ee_pose.GetTranslation()[0];
		_msgDesiredTwist.linear.y  = des_ee_pose.GetTranslation()[1];
		_msgDesiredTwist.linear.z  = des_ee_pose.GetTranslation()[2];
		_msgDesiredTwist.angular.x = 0;
		_msgDesiredTwist.angular.y = 0;
		_msgDesiredTwist.angular.z = 0;

		des_ee_pose.GetOrientation().GetQuaternionRepresentation(des_ee_quaternion);
		_msgDesiredOrientation.w = des_ee_quaternion[0];
		_msgDesiredOrientation.x = des_ee_quaternion[1];
		_msgDesiredOrientation.y = des_ee_quaternion[2];
		_msgDesiredOrientation.z = des_ee_quaternion[3];

		std::cout<<"\nDesired_ee_pose Translation:"<<endl;
		for(int i =0;i<3;i++)
			cout<<des_ee_pose.GetTranslation()[i]<< ", ";
		std::cout<<"\nDesired_ee_pose Quaternion:"<<endl;
		for(int i =0;i<4;i++)
			cout<<des_ee_quaternion[i]<<", ";


		_msgDesiredPose.position.x  = des_ee_pose.GetTranslation()[0];
		_msgDesiredPose.position.y  = des_ee_pose.GetTranslation()[1];
		_msgDesiredPose.position.z  = des_ee_pose.GetTranslation()[2];
		_msgDesiredPose.orientation.w = des_ee_quaternion[0];
		_msgDesiredPose.orientation.x = des_ee_quaternion[1];
		_msgDesiredPose.orientation.y = des_ee_quaternion[2];
		_msgDesiredPose.orientation.z = des_ee_quaternion[3];		

		desiredtwist.publish(_msgDesiredTwist);
		desiredOrientation.publish(_msgDesiredOrientation);
//		desiredPose.publish(_msgDesiredPose);



		ros::spinOnce();
	
	loop_rate.sleep();
	

	++count;

	}


}