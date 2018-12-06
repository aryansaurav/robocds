#include "robocds.h"
#include "GMR.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mathlib/MathLib.h"
#include "Utils.h"
//#include "mathlib_eigen_conversions.h"


#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <dynamic_reconfigure/server.h>
#include <robocds/hand_visualizerConfig.h>



#include <sstream>

#define DOF_HAND 5
#define DOF_JOINTS 16

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

//------------------ THIS CODE IS PROGRAMMED TO CHECK THE HAND FINGER MOTION 
//------------------ WITH DISTANCE FROM OBJECT BEING UNDER MANUAL CONTROL



	double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

    double hand_zero[DOF_JOINTS] = 
    	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


	float alpha = 1;
	float beta = 1;
	double dist[1] = {0.5};
	double snextpos_fingers[DOF_HAND];
	std_msgs::Float64 temp_finger;



	void callback(robocds::hand_visualizerConfig &config, uint32_t level) 
	{
 	
 	ROS_INFO("Reconfigure Request: %f %f %f", 
    	        config.distance, config.beta, config.alpha);
  

  	beta = config.beta;
  	alpha = config.alpha;
  	dist[0] = alpha*config.distance;
	}




int main(int argc, char **argv)

{	
	int grasp_type = 1;
	GMR *coupling_fingers;
	std::cout<<"Enter grasp type (1 for simple, 2 for lateral): ";
	std::cin>>grasp_type;
	if(grasp_type ==1)
		 coupling_fingers= new GMR("data/cplGMM_pos_finger_simple.txt");
	else
		 coupling_fingers= new GMR("data/cplGMM_pos_finger_lateral.txt");
//--------------INITIALIZE COUPLING_FINGERS GMR ----------------------
	std::vector<int> in_dim2, out_dim2;

	in_dim2.resize(1);
	out_dim2.resize(5);

//	if(in_dim.size() + out_dim.size() != coupling->getnVar())
//		return false;

	for(unsigned int i=0;i<in_dim2.size();i++)
		in_dim2[i] = i;

	for(unsigned int i=0;i<out_dim2.size();i++)
		out_dim2[i] = i + in_dim2.size();


	coupling_fingers->initGMR(in_dim2,out_dim2);

	coupling_fingers->printInfo();


//------------ROS NODES, PUBLISHERS AND SUBSCRIBERS -------------------

	ros::init(argc, argv, "hand_visualization_node");
	dynamic_reconfigure::Server<robocds::hand_visualizerConfig> srv;
  	dynamic_reconfigure::Server<robocds::hand_visualizerConfig>::CallbackType f;
  	f = boost::bind(&callback, _1, _2);
  	srv.setCallback(f);	
	
	ros::NodeHandle n1;

//	ros::Publisher joint_cmd_pub  = n1.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd", 3);

	ros::Publisher thumb0 = n1.advertise<std_msgs::Float64>("/allegro/joint1_position_controller/command", 1);
	ros::Publisher thumb1 = n1.advertise<std_msgs::Float64>("/allegro/joint2_position_controller/command", 1);
	ros::Publisher thumb2 = n1.advertise<std_msgs::Float64>("/allegro/joint3_position_controller/command", 1);
	ros::Publisher thumb3 = n1.advertise<std_msgs::Float64>("/allegro/joint4_position_controller/command", 1);
	ros::Publisher finger10 = n1.advertise<std_msgs::Float64>("/allegro/joint5_position_controller/command", 1);
	ros::Publisher finger11 = n1.advertise<std_msgs::Float64>("/allegro/joint6_position_controller/command", 1);
	ros::Publisher finger12 = n1.advertise<std_msgs::Float64>("/allegro/joint7_position_controller/command", 1);
	ros::Publisher finger13 = n1.advertise<std_msgs::Float64>("/allegro/joint8_position_controller/command", 1);
	ros::Publisher finger20 = n1.advertise<std_msgs::Float64>("/allegro/joint9_position_controller/command", 1);
	ros::Publisher finger21 = n1.advertise<std_msgs::Float64>("/allegro/joint10_position_controller/command", 1);
	ros::Publisher finger22 = n1.advertise<std_msgs::Float64>("/allegro/joint11_position_controller/command", 1);
	ros::Publisher finger23 = n1.advertise<std_msgs::Float64>("/allegro/joint12_position_controller/command", 1);
	ros::Publisher finger30 = n1.advertise<std_msgs::Float64>("/allegro/joint13_position_controller/command", 1);
	ros::Publisher finger31 = n1.advertise<std_msgs::Float64>("/allegro/joint14_position_controller/command", 1);
	ros::Publisher finger32 = n1.advertise<std_msgs::Float64>("/allegro/joint15_position_controller/command", 1);
	ros::Publisher finger33 = n1.advertise<std_msgs::Float64>("/allegro/joint16_position_controller/command", 1);



// 	ros::Publisher desiredPose = n1.advertise<geometry_msgs::Pose>("/lwr/joint_controllers/command_pos", 1000);


//	ros::Subscriber subRealTwist = n1.subscribe("/lwr/ee_vel", 1, updateRealTwist);
//    ros::Subscriber joint_state_sub = n1.subscribe("/allegroHand_0/joint_states", 1, updateJointStates);
	ros:: Rate loop_rate(10);



	while(ros::ok())
	{


		coupling_fingers->getGMROutput(dist, snextpos_fingers);

		std::cout<<"\n Next Finger positions:"<<endl;
		for(int i =0;i<DOF_HAND;i++)
			cout<<snextpos_fingers[i]<< ", ";

		temp_finger.data = beta*snextpos_fingers[0];
		thumb0.publish(temp_finger);

		temp_finger.data = beta*snextpos_fingers[1];
		thumb1.publish(temp_finger);

		temp_finger.data = beta*snextpos_fingers[2];
		finger12.publish(temp_finger);
		finger22.publish(temp_finger);
		finger32.publish(temp_finger);
		thumb2.publish(temp_finger);


		temp_finger.data = beta*snextpos_fingers[3];
		finger13.publish(temp_finger);
		finger23.publish(temp_finger);
		finger33.publish(temp_finger);
		thumb3.publish(temp_finger);

		temp_finger.data = beta*snextpos_fingers[4];
		finger11.publish(temp_finger);
		finger21.publish(temp_finger);
		finger31.publish(temp_finger);	



		ros::spinOnce();
		loop_rate.sleep();
	}


}